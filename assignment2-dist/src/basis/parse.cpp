#include "parse.h"
#include <map>
#include "3d/Mesh.hpp"
using namespace std;
using namespace FW;


namespace {

    // read in dim-dimensional control points into a vector
    vector<Vec3f> readCps(istream &in, unsigned dim)
    {    
        // number of control points    
        unsigned n;
        in >> n;

        cerr << "  " << n << " cps" << endl;
    
        // vector of control points
        vector<Vec3f> cps(n);

        char delim;
        float x;
        float y;
        float z;

        for( unsigned i = 0; i < n; ++i )
        {
            switch (dim)
            {
            case 2:
                in >> delim;
                in >> x;
                in >> y;
                cps[i] = Vec3f( x, y, 0 );
                in >> delim;
                break;
            case 3:
                in >> delim;
                in >> x;
                in >> y;
                in >> z;
                cps[i] = Vec3f( x, y, z );
                in >> delim;
                break;            
            default:
                abort();
            }
        }

        return cps;
    }

	Vec4f slerp(float t, Vec4f a, Vec4f b)
	{
		float omega = FW::acos(clamp(FW::abs(dot(a, b)), -1.0f, 1.0f));
		if (dot(a, b) < 0)
			b = -b;
		if (omega == 0)
			return a;
		return
			FW::sin((1 - t) * omega) / FW::sin(omega) * a +
			FW::sin(t * omega) / FW::sin(omega) * b;
	}

	Vec4f quatInverse(Vec4f q) {
		q.x = -q.x;
		q.y = -q.y;
		q.z = -q.z;
		return q;
	}

	Vec4f quatMult(Vec4f a, Vec4f b) {
		return Vec4f(a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y, a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x, a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w, a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z);
	}

	// read in dim-dimensional control points into a vector
	vector<std::array<Vec4f,4>> readQuaternions(istream &in)
	{
		// number of control points    
		unsigned n;
		in >> n;

		cerr << "  " << n << " cps" << endl;

		// vector of control points
		vector<Vec4f> cps(n);

		char delim;

		for (unsigned i = 0; i < n; ++i)
		{
			in >> delim;
			in >> cps[i].x >> cps[i].y >> cps[i].z >> cps[i].w;
			cps[i].normalize();
			in >> delim;
		}
		vector<std::array<Vec4f, 4>> result(n);

		auto clampIdx = [&](unsigned index)->unsigned { while (index < 0)index += n; while (index >= n)index -= n; return index; };

		for (unsigned start = 0; start < n; ++start) {
			result[start][0] = cps[start];
			result[start][1] = quatMult(slerp(.2, Vec4f(.0, .0, .0, 1.), quatMult(cps[clampIdx(start + 1)], quatInverse(cps[clampIdx(start - 1)]))), cps[start]);
			result[start][2] = quatMult(slerp(.2, Vec4f(.0, .0, .0, 1.), quatMult(cps[clampIdx(start)], quatInverse(cps[clampIdx(start + 2)]))), cps[clampIdx(start + 1)]);
			result[start][3] = cps[clampIdx(start + 1)];
		}
		return result;
	}
}



bool parseFile(istream &in,
               vector<vector<Vec3f> > &ctrlPoints, 
               vector<Curve>             &curves,
               vector<string>            &curveNames,
               vector<Surface>           &surfaces,
               vector<string>            &surfaceNames,
			   cameraPath				 &camPath,
			   bool						adaptivetessellation,
			   float					errorbound,
			   float					minstep,
			   string					filepath)
{
    ctrlPoints.clear();
    curves.clear();
    curveNames.clear();
    surfaces.clear();
    surfaceNames.clear();    
	camPath = cameraPath();
    
    string objType;
	vector<array<Vec4f,4>> quaternions;

    // For looking up curve indices by name
    map<string,unsigned> curveIndex;

    // For looking up surface indices by name
    map<string,unsigned> surfaceIndex;
        
    // For storing dimension of curve
    vector<unsigned> dims;

    unsigned counter = 0;
    
    while (in >> objType) 
    {
        cerr << ">object " << counter++ << endl;
        string objName;
        in >> objName;

        bool named = (objName != ".");
        
        vector<Vec3f> cpsToAdd;
        
        if (curveIndex.find(objName) != curveIndex.end() ||
            surfaceIndex.find(objName) != surfaceIndex.end())
        {
            cerr << "error, [" << objName << "] already exists" << endl;
            return false;
        }

        unsigned steps;

        if (objType == "bez2")
        {
            in >> steps;
            cerr << " reading bez2 " << "[" << objName << "]" << endl;
			curves.push_back(evalBezier(cpsToAdd = readCps(in, 2), steps, adaptivetessellation, errorbound, minstep));
            curveNames.push_back(objName);
            dims.push_back(2);
            if (named) curveIndex[objName] = dims.size()-1;
            
        }
        else if (objType == "bsp2")
        {
            cerr << " reading bsp2 " << "[" << objName << "]" << endl;
            in >> steps;
			curves.push_back(evalBspline(cpsToAdd = readCps(in, 2), steps, adaptivetessellation, errorbound, minstep));
            curveNames.push_back(objName);
            dims.push_back(2);
            if (named) curveIndex[objName] = dims.size()-1;
        }
        else if (objType == "bez3")
        {
            cerr << " reading bez3 " << "[" << objName << "]" << endl;
            in >> steps;
			curves.push_back(evalBezier(cpsToAdd = readCps(in, 3), steps, adaptivetessellation, errorbound, minstep));
            curveNames.push_back(objName);
            dims.push_back(3);
            if (named) curveIndex[objName] = dims.size()-1;

        }
        else if (objType == "bsp3")
        {
            cerr << " reading bsp3 " << "[" << objName << "]" << endl;
            in >> steps;
			curves.push_back(evalBspline(cpsToAdd = readCps(in, 3), steps, adaptivetessellation, errorbound, minstep));
            curveNames.push_back(objName);
            dims.push_back(3);
            if (named) curveIndex[objName] = dims.size()-1;
        }
		else if (objType == "orientation")
		{
			cerr << " reading camera path orientations" << endl;
			quaternions = readQuaternions(in);
		}
		else if (objType == "camPath")
		{
			string name;
			in >> name;
			cerr << " reading camera path for obj " << "[" << name << "]" << endl;
			FW::String filename = FW::String((filepath + name).c_str());
			auto mesh = importMesh(filename);
			camPath.mesh.reset((Mesh<VertexPNTC>*)mesh);
			camPath.orientationPoints = quaternions;
			camPath.positionPath = curves[curveIndex["pos"]];
			curves[curveIndex["pos"]] = camPath.positionPath;
			camPath.loaded = true;
		}
        else if (objType == "srev")
        {
            cerr << " reading srev " << "[" << objName << "]" << endl;
            in >> steps;

            // Name of the profile curve
            string profName;
            in >> profName;

            cerr << "  profile [" << profName << "]" << endl;
            
            map<string,unsigned>::const_iterator it = curveIndex.find(profName);

            // Failure checks
            if (it == curveIndex.end()) {                
                cerr << "failed: [" << profName << "] doesn't exist!" << endl; return false;
            }
            if (dims[it->second] != 2) {
                cerr << "failed: [" << profName << "] isn't 2d!" << endl; return false;
            }

            // Make the surface
            surfaces.push_back( makeSurfRev( curves[it->second], steps ) );
            surfaceNames.push_back(objName);
            if (named) surfaceIndex[objName] = surfaceNames.size()-1;
        }
        else if (objType == "gcyl")
        {
            cerr << " reading gcyl " << "[" << objName << "]" << endl;
            
            // Name of the profile curve and sweep curve
            string profName, sweepName;
            in >> profName >> sweepName;

            cerr << "  profile [" << profName << "], sweep [" << sweepName << "]" << endl;

            map<string,unsigned>::const_iterator itP, itS;

            // Failure checks for profile
            itP = curveIndex.find(profName);
            
            if (itP == curveIndex.end()) {                
                cerr << "failed: [" << profName << "] doesn't exist!" << endl; return false;
            }
            if (dims[itP->second] != 2) {
                cerr << "failed: [" << profName << "] isn't 2d!" << endl; return false;
            }

            // Failure checks for sweep
            itS = curveIndex.find(sweepName);
            if (itS == curveIndex.end()) {                
                cerr << "failed: [" << sweepName << "] doesn't exist!" << endl; return false;
            }

            // Make the surface
            surfaces.push_back( makeGenCyl( curves[itP->second], curves[itS->second] ) );
            surfaceNames.push_back(objName);
            if (named) surfaceIndex[objName] = surfaceNames.size()-1;

        }
        else if (objType == "circ")
        {
            cerr << " reading circ " << "[" << objName << "]" << endl;

            float rad;
            in >> steps >> rad;
            cerr << "  radius [" << rad << "]" << endl;

            curves.push_back( evalCircle(rad, steps) );
            curveNames.push_back(objName);
            dims.push_back(2);
            if (named) curveIndex[objName] = dims.size()-1;
        }
        else
        {
            cerr << "failed: type " << objType << " unrecognized." << endl;
            return false;
        }

        ctrlPoints.push_back(cpsToAdd);
    }

    return true;
}




