/***********************************************************************
VolcanoOverlay.h - Volcano detection and lava flow visual overlay
***********************************************************************/

#ifndef _VolcanoOverlay_h_
#define _VolcanoOverlay_h_

#include "../KinectProjector/KinectProjector.h"

struct Volcano {
	ofVec2f kinectPos;
	ofVec2f craterCenter;
	float peakElevation;
	float craterDepth;
	float lastSeenTime;
	bool active;
	std::vector<float> flowAngles;
};

struct LavaParticle {
	ofVec2f kinectPos;
	ofVec2f vel;
	float age;
	float maxAge;
	bool cooled;
	bool erupting;
	float eruptTime;
};

struct LavaDeposit {
	ofVec2f kinectPos;
	float radius;
	float mass;
	float age;
	float lastGrowTime;
};

class CVolcanoOverlay
{
public:
	CVolcanoOverlay();
	virtual ~CVolcanoOverlay();

	void setup(std::shared_ptr<KinectProjector> const& k);
	void update();
	void drawProjectorWindow();

	void setEnabled(bool e) { enabled = e; }
	bool isEnabled() { return enabled; }

	void setProjectorRes(ofVec2f& PR);
	void setKinectRes(ofVec2f& KR);
	void setKinectROI(ofRectangle& KROI);

private:
	std::shared_ptr<KinectProjector> kinectProjector;

	void detectVolcanoes();
	void updateLava();
	void updateDeposits();
	void spawnLava(Volcano& v);
	void drawCraters();
	void drawLava();
	void drawDeposits();

	bool enabled;

	ofVec2f projRes;
	ofVec2f kinectRes;
	ofRectangle kinectROI;

	ofFbo fboVolcano;

	std::vector<Volcano> volcanoes;
	std::vector<LavaParticle> lavaParticles;
	std::vector<LavaDeposit> lavaDeposits;

	float lastDetectionTime;
	float detectionInterval;

	float peakElevationThreshold;
	float calderaCheckRadius;
	float calderaDepthThreshold;
	float baseCheckRadius;
	float baseDropThreshold;
	float minBaseElevation;
	float rimContinuityRatio;

	int maxParticlesPerVolcano;
	float particleSpawnRate;
	float lastSpawnTime;

	float depositMergeRadius;
	float depositMaxRadius;
	float depositFadeTime;
	int numFlowChannels;
};

#endif
