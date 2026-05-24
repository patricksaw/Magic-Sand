/***********************************************************************
TreasureGameController.cpp - Controller for a Sandbox treasure hunt game
***********************************************************************/

#include "TreasureGameController.h"

CTreasureGameController::CTreasureGameController()
{
	CurrentGameSequence = 0;
	treasuresFound = 0;
	totalTreasures = 0;
	buryDepth = 15.0f;
	discoveryThreshold = 5.0f;
	gameDuration = 120.0f;
	freePlayMode = false;
	freePlayMinBury = 5.0f;
	freePlayMaxBury = 30.0f;
	freePlayTreasureCount = 5;
	debugOn = false;
	LastTimeEvent = 0;
	gameStartTime = 0;

	GameSequence.push_back(GAME_STATE_IDLE);
	GameSequenceTimings.push_back(-1);
}

CTreasureGameController::~CTreasureGameController()
{
}

void CTreasureGameController::setup(std::shared_ptr<KinectProjector> const& k)
{
	kinectProjector = k;

	ofTrueTypeFont::setGlobalDpi(72);
	scoreFont.loadFont("verdana.ttf", 48);
	titleFont.loadFont("verdana.ttf", 96);
}

void CTreasureGameController::update()
{
	float now = ofGetElapsedTimef();
	eGameState sequence = GameSequence[CurrentGameSequence];

	if (sequence == GAME_STATE_PLAYING)
	{
		if (!kinectProjector->isImageStabilized())
			return;

		checkForDiscovery();

		// Update particles
		for (int i = particles.size() - 1; i >= 0; i--)
		{
			particles[i].pos += particles[i].vel;
			particles[i].vel.y -= 0.05f;
			particles[i].life -= ofGetLastFrameTime();
			if (particles[i].life <= 0)
				particles.erase(particles.begin() + i);
		}

		if (freePlayMode)
		{
			fboTreasure.begin();
			ofClear(255, 255, 255, 0);
			ofPushStyle();
			drawTreasureEffects();
			ofPopStyle();
			fboTreasure.end();
		}
		else
		{
			// Check win condition
			if (treasuresFound >= totalTreasures)
			{
				CurrentGameSequence++;
				if (CurrentGameSequence >= GameSequence.size())
					CurrentGameSequence = GameSequence.size() - 1;
				InitiateGameSequence();
				return;
			}

			fboTreasure.begin();
			ofClear(255, 255, 255, 0);
			ofPushStyle();

			drawHintParticles();
			drawTreasureEffects();

			float elapsed = now - gameStartTime;
			float remaining = gameDuration - elapsed;
			if (remaining < 0) remaining = 0;

			std::string statusText = "Found: " + ofToString(treasuresFound) + " / " + ofToString(totalTreasures);
			std::string timeText = "Time: " + ofToString((int)remaining) + "s";

			ofSetColor(255, 255, 255, 200);
			float textX = projROI.x + 20;
			float textY = projROI.y + 50;
			scoreFont.drawString(statusText, textX, textY);
			scoreFont.drawString(timeText, textX, textY + 60);

			ofPopStyle();
			fboTreasure.end();

			// Time's up
			if (remaining <= 0)
			{
				CurrentGameSequence++;
				if (CurrentGameSequence >= GameSequence.size())
					CurrentGameSequence = GameSequence.size() - 1;
				InitiateGameSequence();
			}
		}
	}
	else if (sequence == GAME_STATE_IDLE)
	{
		// Nothing
	}

	int DeltaTime = GameSequenceTimings[CurrentGameSequence];
	if (DeltaTime > 0)
	{
		if (now - LastTimeEvent > DeltaTime)
		{
			CurrentGameSequence++;
			if (CurrentGameSequence >= GameSequence.size())
				CurrentGameSequence = GameSequence.size() - 1;
			InitiateGameSequence();
		}
	}
}

void CTreasureGameController::checkForDiscovery()
{
	for (int idx = 0; idx < treasures.size(); idx++)
	{
		Treasure& t = treasures[idx];
		if (t.found)
			continue;

		float currentElevation = kinectProjector->elevationAtKinectCoord(t.kinectPos.x, t.kinectPos.y);

		if (currentElevation <= t.buriedElevation + discoveryThreshold)
		{
			t.found = true;
			t.foundTime = ofGetElapsedTimef();
			treasuresFound++;

			ofVec2f projPos = kinectProjector->kinectCoordToProjCoord(t.kinectPos.x, t.kinectPos.y);
			for (int i = 0; i < 80; i++)
			{
				Particle p;
				p.pos = projPos;
				float angle = ofRandom(0, TWO_PI);
				float speed = ofRandom(1.0f, 5.0f);
				p.vel = ofVec2f(cos(angle) * speed, sin(angle) * speed);
				p.maxLife = ofRandom(1.5f, 3.0f);
				p.life = p.maxLife;
				p.color = ofColor::fromHsb(ofRandom(25, 50), 220, 255);
				particles.push_back(p);
			}

			if (freePlayMode)
				placeOneTreasure();
		}
	}

	// In free play, clean up old found treasures whose effects have faded
	if (freePlayMode)
	{
		float now = ofGetElapsedTimef();
		for (int i = treasures.size() - 1; i >= 0; i--)
		{
			if (treasures[i].found && (now - treasures[i].foundTime) > 6.0f)
				treasures.erase(treasures.begin() + i);
		}
	}
}

void CTreasureGameController::drawHintParticles()
{
	float now = ofGetElapsedTimef();

	for (auto& t : treasures)
	{
		if (t.found)
			continue;

		ofVec2f projPos = kinectProjector->kinectCoordToProjCoord(t.kinectPos.x, t.kinectPos.y);
		float currentElevation = kinectProjector->elevationAtKinectCoord(t.kinectPos.x, t.kinectPos.y);

		float depthRemaining = currentElevation - t.buriedElevation;
		float proximity = 1.0f - ofClamp(depthRemaining / buryDepth, 0.0f, 1.0f);

		float pulse = (sin(now * 3.0f + t.kinectPos.x) + 1.0f) * 0.5f;
		float radius = 8.0f + proximity * 25.0f;
		int alpha = (int)(30 + proximity * 180 * pulse);

		ofSetColor(255, 200, 50, alpha);
		ofDrawCircle(projPos.x, projPos.y, radius);

		if (proximity > 0.5f)
		{
			ofSetColor(255, 255, 150, (int)(proximity * 200 * pulse));
			ofDrawCircle(projPos.x, projPos.y, radius * 0.4f);
		}
	}
}

void CTreasureGameController::drawTreasureEffects()
{
	float now = ofGetElapsedTimef();

	for (auto& t : treasures)
	{
		if (!t.found)
			continue;

		float timeSinceFound = now - t.foundTime;
		ofVec2f projPos = kinectProjector->kinectCoordToProjCoord(t.kinectPos.x, t.kinectPos.y);

		float fadeTime = 5.0f;
		if (timeSinceFound < fadeTime)
		{
			float progress = timeSinceFound / fadeTime;
			int alpha = (int)(255 * (1.0f - progress));

			float ringRadius = 15.0f + timeSinceFound * 20.0f;
			ofSetColor(255, 215, 0, alpha);
			ofNoFill();
			ofSetLineWidth(3);
			ofDrawCircle(projPos.x, projPos.y, ringRadius);
			ofFill();

			ofSetColor(255, 200, 50, alpha);
			ofDrawCircle(projPos.x, projPos.y, 10.0f * (1.0f - progress * 0.5f));
		}
	}

	for (auto& p : particles)
	{
		float alpha = (p.life / p.maxLife) * 255;
		ofSetColor(p.color, (int)alpha);
		float size = 3.0f * (p.life / p.maxLife);
		ofDrawCircle(p.pos.x, p.pos.y, size);
	}
}

bool CTreasureGameController::findValidBurialSite(ofVec2f& location, float& elevation)
{
	int maxAttempts = 200;
	for (int i = 0; i < maxAttempts; i++)
	{
		float x = ofRandom(kinectROI.getLeft() + 20, kinectROI.getRight() - 20);
		float y = ofRandom(kinectROI.getTop() + 20, kinectROI.getBottom() - 20);

		float elev = kinectProjector->elevationAtKinectCoord(x, y);

		if (elev > 5.0f && elev < 80.0f)
		{
			location = ofVec2f(x, y);
			elevation = elev;
			return true;
		}
	}
	return false;
}

void CTreasureGameController::placeOneTreasure()
{
	ofVec2f loc;
	float elev;
	if (findValidBurialSite(loc, elev))
	{
		Treasure t;
		t.kinectPos = loc;
		float depth = freePlayMode ? ofRandom(freePlayMinBury, freePlayMaxBury) : buryDepth;
		t.buriedElevation = elev - depth;
		t.found = false;
		t.foundTime = 0;
		treasures.push_back(t);
	}
}

void CTreasureGameController::placeTreasures()
{
	treasures.clear();
	treasuresFound = 0;
	particles.clear();

	int count = freePlayMode ? freePlayTreasureCount : totalTreasures;
	for (int i = 0; i < count; i++)
		placeOneTreasure();

	if (!freePlayMode)
		totalTreasures = treasures.size();
}

void CTreasureGameController::drawProjectorWindow()
{
	if (!fboTreasure.isAllocated())
		return;

	eGameState sequence = GameSequence[CurrentGameSequence];
	if (sequence != GAME_STATE_IDLE)
	{
		ofSetColor(255);
		fboTreasure.draw(0, 0);
	}
}

void CTreasureGameController::drawMainWindow(float x, float y, float width, float height)
{
	if (!fboTreasure.isAllocated())
		return;

	eGameState sequence = GameSequence[CurrentGameSequence];
	if (sequence != GAME_STATE_IDLE)
	{
		ofSetColor(255);
		fboTreasure.draw(x, y, width, height);
	}
}

bool CTreasureGameController::StartGame(int difficulty)
{
	if (GameSequence[CurrentGameSequence] != GAME_STATE_IDLE)
	{
		CurrentGameSequence = GameSequence.size() - 1;
		return false;
	}

	freePlayMode = false;
	projROI = kinectProjector->getProjectorActiveROI();
	SetupGameSequence(difficulty);
	CurrentGameSequence = 0;
	InitiateGameSequence();
	return true;
}

bool CTreasureGameController::StartFreePlay()
{
	if (GameSequence[CurrentGameSequence] != GAME_STATE_IDLE)
	{
		CurrentGameSequence = GameSequence.size() - 1;
		return false;
	}

	freePlayMode = true;
	projROI = kinectProjector->getProjectorActiveROI();

	GameSequence.clear();
	GameSequenceTimings.clear();

	GameSequence.push_back(GAME_STATE_PLAYING);
	GameSequenceTimings.push_back(-1);

	GameSequence.push_back(GAME_STATE_IDLE);
	GameSequenceTimings.push_back(-1);

	CurrentGameSequence = 0;
	placeTreasures();
	LastTimeEvent = ofGetElapsedTimef();
	return true;
}

void CTreasureGameController::SetupGameSequence(int difficulty)
{
	GameSequence.clear();
	GameSequenceTimings.clear();

	switch (difficulty)
	{
	case 0:
		totalTreasures = 2;
		buryDepth = 8.0f;
		gameDuration = 120.0f;
		break;
	case 1:
		totalTreasures = 4;
		buryDepth = 12.0f;
		gameDuration = 120.0f;
		break;
	case 2:
		totalTreasures = 6;
		buryDepth = 18.0f;
		gameDuration = 90.0f;
		break;
	case 3:
		totalTreasures = 8;
		buryDepth = 25.0f;
		gameDuration = 60.0f;
		break;
	default:
		totalTreasures = 4;
		buryDepth = 12.0f;
		gameDuration = 120.0f;
		break;
	}

	discoveryThreshold = buryDepth * 0.2f;

	GameSequence.push_back(GAME_STATE_SHOWSPLASHSCREEN);
	GameSequenceTimings.push_back(4);

	GameSequence.push_back(GAME_STATE_PLAYING);
	GameSequenceTimings.push_back(-1);

	GameSequence.push_back(GAME_STATE_SHOWRESULT);
	GameSequenceTimings.push_back(8);

	GameSequence.push_back(GAME_STATE_IDLE);
	GameSequenceTimings.push_back(-1);
}

bool CTreasureGameController::InitiateGameSequence()
{
	eGameState sequence = GameSequence[CurrentGameSequence];

	if (sequence == GAME_STATE_SHOWSPLASHSCREEN)
	{
		fboTreasure.begin();
		ofClear(255, 255, 255, 0);

		std::string title = "TREASURE HUNT";
		std::string sub = ofToString(totalTreasures) + " treasures hidden!";
		std::string sub2 = "Dig to find the golden glow!";

		float tw = titleFont.stringWidth(title);
		float sw = scoreFont.stringWidth(sub);
		float s2w = scoreFont.stringWidth(sub2);
		float cx = projROI.x + projROI.width / 2;
		float cy = projROI.y + projROI.height / 2;

		ofSetColor(255, 215, 0);
		titleFont.drawString(title, cx - tw / 2, cy - 40);
		ofSetColor(255, 255, 255);
		scoreFont.drawString(sub, cx - sw / 2, cy + 40);
		scoreFont.drawString(sub2, cx - s2w / 2, cy + 100);

		fboTreasure.end();
	}
	else if (sequence == GAME_STATE_PLAYING)
	{
		placeTreasures();
		gameStartTime = ofGetElapsedTimef();
	}
	else if (sequence == GAME_STATE_SHOWRESULT)
	{
		fboTreasure.begin();
		ofClear(255, 255, 255, 0);

		float elapsed = ofGetElapsedTimef() - gameStartTime;
		std::string result;
		if (treasuresFound >= totalTreasures)
			result = "ALL FOUND in " + ofToString((int)elapsed) + "s!";
		else
			result = "Found " + ofToString(treasuresFound) + " / " + ofToString(totalTreasures);

		float rw = titleFont.stringWidth(result);
		float cx = projROI.x + projROI.width / 2;
		float cy = projROI.y + projROI.height / 2;

		ofSetColor(255, 215, 0);
		titleFont.drawString(result, cx - rw / 2, cy);

		fboTreasure.end();
	}

	LastTimeEvent = ofGetElapsedTimef();
	return true;
}

bool CTreasureGameController::isIdle()
{
	return GameSequence[CurrentGameSequence] == GAME_STATE_IDLE;
}

void CTreasureGameController::stopGame()
{
	if (!isIdle())
		CurrentGameSequence = GameSequence.size() - 1;
}

void CTreasureGameController::setProjectorRes(ofVec2f& PR)
{
	projRes = PR;
	fboTreasure.allocate(projRes.x, projRes.y, GL_RGBA);
	fboTreasure.begin();
	ofClear(0, 0, 0, 255);
	fboTreasure.end();
}

void CTreasureGameController::setKinectRes(ofVec2f& KR)
{
	kinectRes = KR;
}

void CTreasureGameController::setKinectROI(ofRectangle& KROI)
{
	kinectROI = KROI;
}

void CTreasureGameController::setDebug(bool flag)
{
	debugOn = flag;
}
