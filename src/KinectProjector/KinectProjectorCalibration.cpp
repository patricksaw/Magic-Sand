/***********************************************************************
KinectProjectorCalibration.cpp - KinectProjectorCalibration compute
the calibration of the kinect and projector.
Copyright (c) 2016 Thomas Wolf

--- Adapted from ofxKinectProjectorToolkit by Gene Kogan:
https://github.com/genekogan/ofxKinectProjectorToolkit
Copyright (c) 2014 Gene Kogan

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
with the Magic Sand; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#include "KinectProjectorCalibration.h"
#include <cmath>

ofxKinectProjectorToolkit::ofxKinectProjectorToolkit(ofVec2f sprojRes, ofVec2f skinectRes) {
	projRes = sprojRes;
	kinectRes = skinectRes;
    calibrated = false;
}

void ofxKinectProjectorToolkit::calibrate(vector<ofVec3f> pairsKinect,
                                          vector<ofVec2f> pairsProjector,
                                          bool verbose) {
    int nPairs = pairsKinect.size();

    // Hartley normalization: center and scale both point sets so the DLT
    // matrix columns have comparable magnitude.  Without this, the cross-term
    // columns (kinect_coord * projector_coord) dwarf the linear columns by
    // ~1000x, making the QR solution numerically unstable.

    ofVec3f centW(0, 0, 0);
    for (int i = 0; i < nPairs; i++) centW += pairsKinect[i];
    centW /= (float)nPairs;

    double avgDistW = 0;
    for (int i = 0; i < nPairs; i++)
        avgDistW += (pairsKinect[i] - centW).length();
    avgDistW /= nPairs;
    double sW = sqrt(3.0) / avgDistW;

    ofVec2f centP(0, 0);
    for (int i = 0; i < nPairs; i++) centP += pairsProjector[i];
    centP /= (float)nPairs;

    double avgDistP = 0;
    for (int i = 0; i < nPairs; i++)
        avgDistP += (pairsProjector[i] - centP).length();
    avgDistP /= nPairs;
    double sP = sqrt(2.0) / avgDistP;

    if (verbose)
        cout << "DLT normalization: centW=(" << centW.x << "," << centW.y << "," << centW.z
             << ") sW=" << sW << "  centP=(" << centP.x << "," << centP.y << ") sP=" << sP << endl;

    A.set_size(nPairs * 2, 11);
    y.set_size(nPairs * 2, 1);

    for (int i = 0; i < nPairs; i++) {
        double wx = sW * (pairsKinect[i].x - centW.x);
        double wy = sW * (pairsKinect[i].y - centW.y);
        double wz = sW * (pairsKinect[i].z - centW.z);
        double px = sP * (pairsProjector[i].x - centP.x);
        double py = sP * (pairsProjector[i].y - centP.y);

        A(2*i, 0) = wx;      A(2*i, 1) = wy;      A(2*i, 2) = wz;      A(2*i, 3) = 1;
        A(2*i, 4) = 0;       A(2*i, 5) = 0;        A(2*i, 6) = 0;       A(2*i, 7) = 0;
        A(2*i, 8) = -wx*px;  A(2*i, 9) = -wy*px;   A(2*i, 10) = -wz*px;

        A(2*i+1, 0) = 0;       A(2*i+1, 1) = 0;        A(2*i+1, 2) = 0;       A(2*i+1, 3) = 0;
        A(2*i+1, 4) = wx;      A(2*i+1, 5) = wy;       A(2*i+1, 6) = wz;      A(2*i+1, 7) = 1;
        A(2*i+1, 8) = -wx*py;  A(2*i+1, 9) = -wy*py;   A(2*i+1, 10) = -wz*py;

        y(2*i, 0) = px;
        y(2*i+1, 0) = py;
    }

    dlib::qr_decomposition<dlib::matrix<double, 0, 11> > qrd(A);
    dlib::matrix<double, 11, 1> xn = qrd.solve(y);
    if (verbose) cout << "xn (normalized): " << xn << endl;

    // Denormalize: H_orig = Tp_inv * H_norm * Tw
    // H_norm is the 3x4 projection matrix in normalized coordinates.
    // Tp_inv undoes 2D normalization, Tw applies 3D normalization.
    double Hn[3][4] = {
        {xn(0,0), xn(1,0), xn(2,0), xn(3,0)},
        {xn(4,0), xn(5,0), xn(6,0), xn(7,0)},
        {xn(8,0), xn(9,0), xn(10,0), 1.0}
    };

    double HT[3][4];
    for (int i = 0; i < 3; i++) {
        HT[i][0] = Hn[i][0] * sW;
        HT[i][1] = Hn[i][1] * sW;
        HT[i][2] = Hn[i][2] * sW;
        HT[i][3] = Hn[i][3] - sW * (Hn[i][0]*centW.x + Hn[i][1]*centW.y + Hn[i][2]*centW.z);
    }

    double Ho[3][4];
    for (int j = 0; j < 4; j++) {
        Ho[0][j] = HT[0][j] / sP + centP.x * HT[2][j];
        Ho[1][j] = HT[1][j] / sP + centP.y * HT[2][j];
        Ho[2][j] = HT[2][j];
    }

    double denom = Ho[2][3];
    if (fabs(denom) < 1e-12) denom = 1e-12;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
            Ho[i][j] /= denom;

    x(0,0) = Ho[0][0]; x(1,0) = Ho[0][1]; x(2,0) = Ho[0][2]; x(3,0) = Ho[0][3];
    x(4,0) = Ho[1][0]; x(5,0) = Ho[1][1]; x(6,0) = Ho[1][2]; x(7,0) = Ho[1][3];
    x(8,0) = Ho[2][0]; x(9,0) = Ho[2][1]; x(10,0) = Ho[2][2];

    if (verbose) cout << "x (denormalized): " << x << endl;

    projMatrice = ofMatrix4x4(x(0,0), x(1,0), x(2,0), x(3,0),
                              x(4,0), x(5,0), x(6,0), x(7,0),
                              x(8,0), x(9,0), x(10,0), 1,
                              0, 0, 0, 1);
    calibrated = true;
}

ofMatrix4x4 ofxKinectProjectorToolkit::getProjectionMatrix() {
    return projMatrice;
}

ofVec2f ofxKinectProjectorToolkit::getProjectedPoint(ofVec3f worldPoint) {
    ofVec4f pts = ofVec4f(worldPoint);
    pts.w = 1;
    ofVec4f rst = projMatrice*(pts);
    ofVec2f projectedPoint(rst.x/rst.z, rst.y/rst.z);
    return projectedPoint;
}

vector<double> ofxKinectProjectorToolkit::getCalibration()
{
    vector<double> coefficients;
    for (int i=0; i<11; i++) {
        coefficients.push_back(x(i, 0));
    }
    return coefficients;
}

bool ofxKinectProjectorToolkit::loadCalibration(string path){
    ofXml xml;
    if (!xml.load(path))
        return false;
    auto calibration = xml.getChild("CALIBRATION");
    if (!calibration)
        return false;
    auto resolutions = calibration.getChild("RESOLUTIONS");
    if (!resolutions)
        return false;
    ofVec2f sprojRes = resolutions.getChild("PROJECTOR").getValue<ofVec2f>();
    ofVec2f skinectRes = resolutions.getChild("KINECT").getValue<ofVec2f>();
    if (sprojRes!=projRes || skinectRes!=kinectRes)
        return false;
    auto coefficients = calibration.getChild("COEFFICIENTS");
    if (!coefficients)
        return false;
    for (int i=0; i<11; i++) {
        x(i, 0) = coefficients.getChild("COEFF"+ofToString(i)).getValue<float>();
    }
    projMatrice = ofMatrix4x4(x(0,0), x(1,0), x(2,0), x(3,0),
                              x(4,0), x(5,0), x(6,0), x(7,0),
                              x(8,0), x(9,0), x(10,0), 1,
                              0, 0, 0, 1);
    calibrated = true;
    return true;
}

bool ofxKinectProjectorToolkit::saveCalibration(string path){
    ofXml xml;
    auto calibration = xml.appendChild("CALIBRATION");
    auto resolutions = calibration.appendChild("RESOLUTIONS");
    auto projNode = resolutions.appendChild("PROJECTOR");
    projNode.set(projRes);
    auto kinectNode = resolutions.appendChild("KINECT");
    kinectNode.set(kinectRes);
    auto coefficients = calibration.appendChild("COEFFICIENTS");
    for (int i=0; i<11; i++) {
        auto coeff = coefficients.appendChild("COEFF"+ofToString(i));
        coeff.set(x(i, 0));
    }
    return xml.save(path);
}


