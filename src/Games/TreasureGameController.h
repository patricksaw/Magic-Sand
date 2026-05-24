/***********************************************************************
TreasureGameController.h - Controller for a Sandbox treasure hunt game
***********************************************************************/

#ifndef _TreasureGameController_h_
#define _TreasureGameController_h_

#include "../KinectProjector/KinectProjector.h"

struct Treasure {
	ofVec2f kinectPos;
	float buriedElevation;
	bool found;
	float foundTime;
};

class CTreasureGameController
{
public:
	CTreasureGameController();
	virtual ~CTreasureGameController();

	void setup(std::shared_ptr<KinectProjector> const& k);
	void update();

	void drawProjectorWindow();
	void drawMainWindow(float x, float y, float width, float height);

	bool StartGame(int difficulty);
	bool StartFreePlay();
	bool isIdle();
	void stopGame();

	void setProjectorRes(ofVec2f& PR);
	void setKinectRes(ofVec2f& KR);
	void setKinectROI(ofRectangle& KROI);
	void setDebug(bool flag);

private:
	std::shared_ptr<KinectProjector> kinectProjector;

	void placeTreasures();
	void placeOneTreasure();
	bool findValidBurialSite(ofVec2f& location, float& elevation);
	void drawTreasureEffects();
	void drawHintParticles();
	void checkForDiscovery();

	ofVec2f projRes;
	ofVec2f kinectRes;
	ofRectangle kinectROI;
	ofRectangle projROI;

	ofFbo fboTreasure;

	ofTrueTypeFont scoreFont;
	ofTrueTypeFont titleFont;

	std::vector<Treasure> treasures;
	int treasuresFound;
	int totalTreasures;
	float gameStartTime;
	float gameDuration;

	float buryDepth;
	float discoveryThreshold;

	struct Particle {
		ofVec2f pos;
		ofVec2f vel;
		float life;
		float maxLife;
		ofColor color;
	};
	std::vector<Particle> particles;

	enum eGameState {
		GAME_STATE_IDLE,
		GAME_STATE_SHOWSPLASHSCREEN,
		GAME_STATE_PLAYING,
		GAME_STATE_SHOWRESULT
	};

	std::vector<eGameState> GameSequence;
	std::vector<int> GameSequenceTimings;
	int CurrentGameSequence;
	float LastTimeEvent;

	bool InitiateGameSequence();
	void SetupGameSequence(int difficulty);

	bool freePlayMode;
	float freePlayMinBury;
	float freePlayMaxBury;
	int freePlayTreasureCount;

	bool debugOn;
};

#endif
