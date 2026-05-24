/***********************************************************************
VolcanoOverlay.cpp - Volcano detection and lava flow visual overlay
***********************************************************************/

#include "VolcanoOverlay.h"

CVolcanoOverlay::CVolcanoOverlay()
{
	enabled = false;
	lastDetectionTime = 0;
	detectionInterval = 0.5f;
	peakElevationThreshold = 50.0f;
	calderaCheckRadius = 8.0f;
	calderaDepthThreshold = 15.0f;
	baseCheckRadius = 35.0f;
	baseDropThreshold = 20.0f;
	minBaseElevation = 10.0f;
	rimContinuityRatio = 0.35f;
	maxParticlesPerVolcano = 200;
	particleSpawnRate = 0.04f;
	lastSpawnTime = 0;
	depositMergeRadius = 6.0f;
	depositMaxRadius = 22.0f;
	depositFadeTime = 45.0f;
	numFlowChannels = 4;
}

CVolcanoOverlay::~CVolcanoOverlay()
{
}

void CVolcanoOverlay::setup(std::shared_ptr<KinectProjector> const& k)
{
	kinectProjector = k;
}

void CVolcanoOverlay::update()
{
	if (!enabled || !kinectProjector->isImageStabilized())
	{
		if (!volcanoes.empty() || !lavaParticles.empty() || !lavaDeposits.empty())
		{
			volcanoes.clear();
			lavaParticles.clear();
			lavaDeposits.clear();
		}
		return;
	}

	float now = ofGetElapsedTimef();

	if (now - lastDetectionTime > detectionInterval)
	{
		detectVolcanoes();
		lastDetectionTime = now;
	}

	if (now - lastSpawnTime > particleSpawnRate)
	{
		for (auto& v : volcanoes)
		{
			if (v.active)
				spawnLava(v);
		}
		lastSpawnTime = now;
	}

	updateLava();
	updateDeposits();

	fboVolcano.begin();
	ofClear(255, 255, 255, 0);
	ofPushStyle();
	drawDeposits();
	drawCraters();
	drawLava();
	ofPopStyle();
	fboVolcano.end();
}

void CVolcanoOverlay::detectVolcanoes()
{
	float now = ofGetElapsedTimef();

	float step = 8.0f;
	float margin = baseCheckRadius;
	float roiLeft = kinectROI.getLeft() + margin;
	float roiRight = kinectROI.getRight() - margin;
	float roiTop = kinectROI.getTop() + margin;
	float roiBottom = kinectROI.getBottom() - margin;

	struct Candidate { ofVec2f pos; float elevation; float calderaDepth; ofVec2f craterCenter; };
	std::vector<Candidate> candidates;

	for (float y = roiTop; y < roiBottom; y += step)
	{
		for (float x = roiLeft; x < roiRight; x += step)
		{
			float elev = kinectProjector->elevationAtKinectCoord(x, y);
			if (elev < peakElevationThreshold)
				continue;

			bool isMax = true;
			for (float dy = -step; dy <= step && isMax; dy += step)
			{
				for (float dx = -step; dx <= step && isMax; dx += step)
				{
					if (dx == 0 && dy == 0) continue;
					float neighborElev = kinectProjector->elevationAtKinectCoord(x + dx, y + dy);
					if (neighborElev > elev)
						isMax = false;
				}
			}

			if (isMax)
				candidates.push_back({ofVec2f(x, y), elev, 0, ofVec2f(x, y)});
		}
	}

	float mergeRadius = baseCheckRadius * 1.2f;
	std::vector<Candidate> merged;
	std::vector<bool> used(candidates.size(), false);

	for (size_t i = 0; i < candidates.size(); i++)
	{
		if (used[i]) continue;
		Candidate best = candidates[i];
		used[i] = true;

		for (size_t j = i + 1; j < candidates.size(); j++)
		{
			if (used[j]) continue;
			if (candidates[j].pos.distance(best.pos) < mergeRadius)
			{
				if (candidates[j].elevation > best.elevation)
					best = candidates[j];
				used[j] = true;
			}
		}
		merged.push_back(best);
	}

	std::vector<Candidate> validated;
	int numSamples = 16;

	for (auto& c : merged)
	{
		float innerMin = c.elevation;
		int highRimCount = 0;
		float rimThreshold = c.elevation * 0.7f;

		for (int i = 0; i < numSamples; i++)
		{
			float angle = (float)i / numSamples * TWO_PI;
			float sx = c.pos.x + cos(angle) * calderaCheckRadius;
			float sy = c.pos.y + sin(angle) * calderaCheckRadius;
			float e = kinectProjector->elevationAtKinectCoord(sx, sy);
			if (e < innerMin)
				innerMin = e;
			if (e > rimThreshold)
				highRimCount++;
		}

		float calderaDip = c.elevation - innerMin;
		if (calderaDip < calderaDepthThreshold)
			continue;

		if (highRimCount < (int)(numSamples * rimContinuityRatio))
			continue;

		// Inner ring average must also dip (not just one noisy low sample)
		float innerRingAvg = 0;
		for (int i = 0; i < numSamples; i++)
		{
			float angle = (float)i / numSamples * TWO_PI;
			float sx = c.pos.x + cos(angle) * (calderaCheckRadius * 0.5f);
			float sy = c.pos.y + sin(angle) * (calderaCheckRadius * 0.5f);
			innerRingAvg += kinectProjector->elevationAtKinectCoord(sx, sy);
		}
		innerRingAvg /= numSamples;

		if (c.elevation - innerRingAvg < calderaDepthThreshold * 0.4f)
			continue;

		ofVec2f craterCenter = c.pos;
		float craterMinElev = c.elevation;
		for (int i = 0; i < numSamples; i++)
		{
			float angle = (float)i / numSamples * TWO_PI;
			for (float r = 2.0f; r <= calderaCheckRadius; r += 2.0f)
			{
				float sx = c.pos.x + cos(angle) * r;
				float sy = c.pos.y + sin(angle) * r;
				float e = kinectProjector->elevationAtKinectCoord(sx, sy);
				if (e < craterMinElev)
				{
					craterMinElev = e;
					craterCenter = ofVec2f(sx, sy);
				}
			}
		}

		float baseAvg = 0;
		for (int i = 0; i < numSamples; i++)
		{
			float angle = (float)i / numSamples * TWO_PI;
			float sx = c.pos.x + cos(angle) * baseCheckRadius;
			float sy = c.pos.y + sin(angle) * baseCheckRadius;
			baseAvg += kinectProjector->elevationAtKinectCoord(sx, sy);
		}
		baseAvg /= numSamples;

		if (baseAvg < minBaseElevation)
			continue;
		if (c.elevation - baseAvg < baseDropThreshold)
			continue;

		Candidate vc = c;
		vc.calderaDepth = calderaDip;
		vc.craterCenter = craterCenter;
		validated.push_back(vc);
	}

	float matchRadius = baseCheckRadius;

	for (auto& v : volcanoes)
		v.active = false;

	for (auto& c : validated)
	{
		bool matched = false;
		for (auto& v : volcanoes)
		{
			if (v.kinectPos.distance(c.pos) < matchRadius)
			{
				v.kinectPos = c.pos;
				v.craterCenter = c.craterCenter;
				v.peakElevation = c.elevation;
				v.craterDepth = c.calderaDepth;
				v.lastSeenTime = now;
				v.active = true;
				matched = true;
				break;
			}
		}
		if (!matched)
		{
			Volcano v;
			v.kinectPos = c.pos;
			v.craterCenter = c.craterCenter;
			v.peakElevation = c.elevation;
			v.craterDepth = c.calderaDepth;
			v.lastSeenTime = now;
			v.active = true;
			for (int i = 0; i < numFlowChannels; i++)
				v.flowAngles.push_back(ofRandom(0, TWO_PI));
			volcanoes.push_back(v);
		}
	}

	for (int i = volcanoes.size() - 1; i >= 0; i--)
	{
		if (!volcanoes[i].active && (now - volcanoes[i].lastSeenTime) > 3.0f)
			volcanoes.erase(volcanoes.begin() + i);
	}
}

void CVolcanoOverlay::spawnLava(Volcano& v)
{
	int count = 0;
	for (auto& p : lavaParticles)
	{
		if (p.kinectPos.distance(v.kinectPos) < baseCheckRadius * 2 || p.age < 0.5f)
			count++;
	}
	int room = maxParticlesPerVolcano - count;
	if (room <= 0)
		return;

	int spawnCount = std::min(5, room);
	for (int i = 0; i < spawnCount; i++)
	{
		float angle;
		if (ofRandom(1.0f) < 0.7f && !v.flowAngles.empty())
		{
			int channel = (int)ofRandom(0, v.flowAngles.size());
			angle = v.flowAngles[channel] + ofRandom(-0.4f, 0.4f);
		}
		else
		{
			angle = ofRandom(0, TWO_PI);
		}

		LavaParticle p;
		p.kinectPos = v.craterCenter;

		float eruptSpeed = ofRandom(0.6f, 1.3f);
		p.vel = ofVec2f(cos(angle) * eruptSpeed, sin(angle) * eruptSpeed);

		p.age = 0;
		p.maxAge = ofRandom(10.0f, 20.0f);
		p.cooled = false;
		p.erupting = true;
		p.eruptTime = ofRandom(0.3f, 0.7f);

		lavaParticles.push_back(p);
	}
}

void CVolcanoOverlay::updateLava()
{
	float dt = ofGetLastFrameTime();

	for (int i = lavaParticles.size() - 1; i >= 0; i--)
	{
		LavaParticle& p = lavaParticles[i];
		p.age += dt;

		if (p.age >= p.maxAge)
		{
			lavaParticles.erase(lavaParticles.begin() + i);
			continue;
		}

		if (p.cooled)
			continue;

		if (p.erupting)
		{
			p.kinectPos += p.vel;
			p.vel *= 0.98f;

			if (p.age >= p.eruptTime)
			{
				p.erupting = false;
				p.vel *= 0.3f;
			}

			p.kinectPos.x = ofClamp(p.kinectPos.x, kinectROI.getLeft(), kinectROI.getRight());
			p.kinectPos.y = ofClamp(p.kinectPos.y, kinectROI.getTop(), kinectROI.getBottom());
			continue;
		}

		float elev = kinectProjector->elevationAtKinectCoord(p.kinectPos.x, p.kinectPos.y);

		if (elev < -2.0f)
		{
			p.cooled = true;
			p.vel = ofVec2f(0, 0);

			bool merged = false;
			for (auto& d : lavaDeposits)
			{
				if (d.kinectPos.distance(p.kinectPos) < depositMergeRadius)
				{
					d.mass += 1.0f;
					d.radius = ofClamp(d.radius + 0.4f, 0, depositMaxRadius);
					d.lastGrowTime = ofGetElapsedTimef();
					merged = true;
					break;
				}
			}
			if (!merged)
			{
				LavaDeposit d;
				d.kinectPos = p.kinectPos;
				d.radius = 6.0f;
				d.mass = 1.0f;
				d.age = 0;
				d.lastGrowTime = ofGetElapsedTimef();
				lavaDeposits.push_back(d);
			}
			continue;
		}

		ofVec2f grad = kinectProjector->gradientAtKinectCoord(p.kinectPos.x, p.kinectPos.y);
		ofVec2f downhill = -grad;

		float gradMag = downhill.length();
		if (gradMag > 0.001f)
		{
			downhill.normalize();
			float speed = ofClamp(gradMag * 1.5f, 0.4f, 5.0f);
			p.vel = p.vel * 0.8f + downhill * speed * 0.2f;
		}
		else
		{
			p.vel *= 0.85f;
		}

		p.kinectPos += p.vel;

		p.kinectPos.x = ofClamp(p.kinectPos.x, kinectROI.getLeft(), kinectROI.getRight());
		p.kinectPos.y = ofClamp(p.kinectPos.y, kinectROI.getTop(), kinectROI.getBottom());
	}
}

void CVolcanoOverlay::updateDeposits()
{
	float now = ofGetElapsedTimef();

	for (int i = lavaDeposits.size() - 1; i >= 0; i--)
	{
		LavaDeposit& d = lavaDeposits[i];
		d.age += ofGetLastFrameTime();

		float timeSinceGrow = now - d.lastGrowTime;
		if (timeSinceGrow > depositFadeTime + 20.0f)
		{
			lavaDeposits.erase(lavaDeposits.begin() + i);
			continue;
		}
	}
}

void CVolcanoOverlay::drawCraters()
{
	float now = ofGetElapsedTimef();

	for (auto& v : volcanoes)
	{
		if (!v.active)
			continue;

		ofVec2f projPos = kinectProjector->kinectCoordToProjCoord(v.craterCenter.x, v.craterCenter.y);

		float pulse = (sin(now * 4.0f + v.kinectPos.x * 0.1f) + 1.0f) * 0.5f;
		float innerPulse = (sin(now * 6.0f + v.kinectPos.y * 0.1f) + 1.0f) * 0.5f;

		float outerRadius = 22.0f + pulse * 10.0f;
		ofSetColor(200, 50, 0, (int)(100 + pulse * 50));
		ofDrawCircle(projPos.x, projPos.y, outerRadius);

		float midRadius = 14.0f + innerPulse * 5.0f;
		ofSetColor(255, 100, 0, (int)(150 + innerPulse * 70));
		ofDrawCircle(projPos.x, projPos.y, midRadius);

		float coreRadius = 7.0f + innerPulse * 4.0f;
		ofSetColor(255, 200, 50, (int)(200 + pulse * 55));
		ofDrawCircle(projPos.x, projPos.y, coreRadius);
	}
}

void CVolcanoOverlay::drawLava()
{
	for (auto& p : lavaParticles)
	{
		ofVec2f projPos = kinectProjector->kinectCoordToProjCoord(p.kinectPos.x, p.kinectPos.y);
		float lifeRatio = p.age / p.maxAge;

		ofColor color;
		float size;

		if (p.cooled)
		{
			float coolFade = ofClamp((p.age - 0.5f) / p.maxAge, 0.0f, 1.0f);
			color = ofColor(50, 15, 5, (int)(200 * (1.0f - coolFade)));
			size = 4.0f;
		}
		else if (p.erupting)
		{
			color = ofColor(255, 240, 120, 255);
			size = 5.0f + ofRandom(0, 2.0f);
		}
		else if (lifeRatio < 0.1f)
		{
			color = ofColor(255, 220, 80, 245);
			size = 5.0f + (1.0f - lifeRatio / 0.1f) * 2.0f;
		}
		else if (lifeRatio < 0.25f)
		{
			float t = (lifeRatio - 0.1f) / 0.15f;
			color = ofColor(255, (int)(220 - t * 120), (int)(80 - t * 60), 235);
			size = 5.0f;
		}
		else if (lifeRatio < 0.5f)
		{
			float t = (lifeRatio - 0.25f) / 0.25f;
			color = ofColor(255 - (int)(t * 60), (int)(100 - t * 40), 20, 225);
			size = 4.5f;
		}
		else
		{
			float t = (lifeRatio - 0.5f) / 0.5f;
			color = ofColor((int)(195 - t * 140), (int)(60 - t * 40), 10, (int)(225 - t * 130));
			size = 4.0f + t * 1.0f;
		}

		ofSetColor(color);
		ofDrawCircle(projPos.x, projPos.y, size);
	}
}

void CVolcanoOverlay::drawDeposits()
{
	float now = ofGetElapsedTimef();

	for (auto& d : lavaDeposits)
	{
		ofVec2f projPos = kinectProjector->kinectCoordToProjCoord(d.kinectPos.x, d.kinectPos.y);

		float timeSinceGrow = now - d.lastGrowTime;
		float alpha = 1.0f;
		if (timeSinceGrow > depositFadeTime)
			alpha = 1.0f - (timeSinceGrow - depositFadeTime) / 20.0f;
		alpha = ofClamp(alpha, 0.0f, 1.0f);

		float coolT = ofClamp(timeSinceGrow / 12.0f, 0.0f, 1.0f);
		int r = (int)(110 - coolT * 55);
		int g = (int)(45 - coolT * 15);
		int b = (int)(25 - coolT * 5);

		ofSetColor(r, g, b, (int)(200 * alpha));
		ofDrawCircle(projPos.x, projPos.y, d.radius);

		if (timeSinceGrow < 6.0f)
		{
			float warmth = 1.0f - timeSinceGrow / 6.0f;
			ofSetColor((int)(160 * warmth + 50), (int)(40 * warmth + 20), 15, (int)(170 * alpha * warmth));
			ofDrawCircle(projPos.x, projPos.y, d.radius * 0.55f);
		}
	}
}

void CVolcanoOverlay::drawProjectorWindow()
{
	if (!enabled || !fboVolcano.isAllocated())
		return;

	ofSetColor(255);
	fboVolcano.draw(0, 0);
}

void CVolcanoOverlay::setProjectorRes(ofVec2f& PR)
{
	projRes = PR;
	fboVolcano.allocate(projRes.x, projRes.y, GL_RGBA);
	fboVolcano.begin();
	ofClear(0, 0, 0, 0);
	fboVolcano.end();
}

void CVolcanoOverlay::setKinectRes(ofVec2f& KR)
{
	kinectRes = KR;
}

void CVolcanoOverlay::setKinectROI(ofRectangle& KROI)
{
	kinectROI = KROI;
}
