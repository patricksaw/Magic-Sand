/***********************************************************************
Main.cpp
Copyright (c) 2016-2017 Thomas Wolf and Rasmus R. Paulsen (people.compute.dtu.dk/rapa)

This file is part of the Magic Sand.

The Magic Sand is free software; you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The Magic Sand is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Augmented Reality Sandbox; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/


#include "ofMain.h"
#include "ofApp.h"
#include "GLFW/glfw3.h"
#include <signal.h>
#include <atomic>
#include <unistd.h>
#include <execinfo.h>

const std::string MagicSandVersion = "1.5.4.2";

static std::weak_ptr<ofApp> g_appRef;
static std::atomic<bool> g_cleaningUp{false};

static void crashHandler(int signum) {
	fprintf(stderr, "\n=== CRASH (signal %d) ===\n", signum);
	void* frames[64];
	int count = backtrace(frames, 64);
	backtrace_symbols_fd(frames, count, STDERR_FILENO);
	fprintf(stderr, "=== END CRASH ===\n");

	auto app = g_appRef.lock();
	if (app && app->kinectProjector) {
		app->kinectProjector->forceCloseKinect();
	}
	_exit(1);
}

static void kinectSignalHandler(int signum) {
	if (g_cleaningUp.exchange(true)) {
		_exit(1);
	}
	fprintf(stderr, "\nCaught signal %d, saving settings and closing kinect...\n", signum);
	auto app = g_appRef.lock();
	if (app) {
		if (app->kinectProjector) {
			app->kinectProjector->saveCalibrationAndSettings();
			app->kinectProjector->forceCloseKinect();
		}
		if (app->sandSurfaceRenderer) {
			app->sandSurfaceRenderer->saveSettings();
		}
	}
	fprintf(stderr, "Done. Exiting.\n");
	_exit(0);
}

bool setWindowDimensions(ofGLFWWindowSettings& settings, int windowsNum) {
	int count;
	GLFWmonitor** monitors = glfwGetMonitors(&count);
	cout << "Number of screens found: " << count << endl;
	if (count > windowsNum) {
		int xM; int yM;
		glfwGetMonitorPos(monitors[windowsNum], &xM, &yM);
		const GLFWvidmode * desktopMode = glfwGetVideoMode(monitors[windowsNum]);

        cout << "Monitor " << windowsNum << " size: " << desktopMode->width << "x" << desktopMode->height << endl;

		if (windowsNum == 0)
		{
			float xscale = 1.0f, yscale = 1.0f;
			glfwGetMonitorContentScale(monitors[windowsNum], &xscale, &yscale);
			int w = (int)(desktopMode->width / xscale * 0.7);
			int h = (int)(desktopMode->height / yscale * 0.7);
			if (w < 980) w = 980;
			if (h < 620) h = 620;
			settings.setSize(w, h);
		}
		else
		{
			settings.setSize(desktopMode->width, desktopMode->height);
		}

		settings.setPosition(glm::vec2(xM, yM));

		return true;
	}
	else {
		settings.setSize(1600, 800);
		settings.setPosition(glm::vec2(0, 0));
		return false;
	}

}

//========================================================================
int main() {

	int count;
	GLFWmonitor** monitors = glfwGetMonitors(&count);
	cout << "Number of screens found: " << count << endl;

	int mainMonitor = 0;
	int projMonitor = 1;

	if (count >= 2) {
		for (int i = 0; i < count; i++) {
			const char* name = glfwGetMonitorName(monitors[i]);
			const GLFWvidmode* mode = glfwGetVideoMode(monitors[i]);
			int xM, yM;
			glfwGetMonitorPos(monitors[i], &xM, &yM);
			cout << "Monitor " << i << ": \"" << (name ? name : "unknown") << "\" "
				 << mode->width << "x" << mode->height << " at (" << xM << "," << yM << ")" << endl;
		}

		// Built-in display is usually at (0,0); pick the other one for the projector
		int x0, y0, x1, y1;
		glfwGetMonitorPos(monitors[0], &x0, &y0);
		glfwGetMonitorPos(monitors[1], &x1, &y1);

		if (x1 == 0 && y1 == 0 && (x0 != 0 || y0 != 0)) {
			mainMonitor = 1;
			projMonitor = 0;
			cout << "Detected: monitor 0 is external (projector), monitor 1 is built-in (main)" << endl;
		} else {
			cout << "Using default: monitor 0 is main, monitor 1 is projector" << endl;
		}
	}

	ofGLFWWindowSettings settings;
    settings.setSize(1600, 800);
    settings.setPosition(glm::vec2(0, 0));
	settings.resizable = true;
	settings.decorated = true;
	settings.title = "Magic-Sand " + MagicSandVersion;
	shared_ptr<ofAppBaseWindow> mainWindow = ofCreateWindow(settings);

	setWindowDimensions(settings, mainMonitor);
	mainWindow->setWindowPosition(settings.getPosition().x + settings.getWidth() / 10,
								  settings.getPosition().y + settings.getHeight() / 10);
    mainWindow->setWindowShape(settings.getWidth(), settings.getHeight());

	setWindowDimensions(settings, projMonitor);
	settings.resizable = false;
	settings.decorated = false;
	settings.windowMode = OF_FULLSCREEN;
	settings.monitor = projMonitor;
	settings.shareContextWith = mainWindow;
	shared_ptr<ofAppBaseWindow> secondWindow = ofCreateWindow(settings);
	secondWindow->setVerticalSync(false);

	shared_ptr<ofApp> mainApp(new ofApp);
	ofAddListener(secondWindow->events().draw, mainApp.get(), &ofApp::drawProjWindow);
	mainApp->projWindow = secondWindow;

	g_appRef = mainApp;
	signal(SIGINT, kinectSignalHandler);
	signal(SIGTERM, kinectSignalHandler);
	signal(SIGSEGV, crashHandler);
	signal(SIGBUS, crashHandler);
	signal(SIGABRT, crashHandler);

	ofRunApp(mainWindow, mainApp);
	ofRunMainLoop();
}
