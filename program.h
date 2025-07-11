#pragma once
#include <SDL3/SDL.h>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <strstream>
#include <vector>

struct Vector2 {
  double x, y;
};

struct Vector3 {
  float x, y, z;
};

struct triangle {
  Vector3 p[3];
};

struct Mat4x4 {
  float m[4][4] = {0};
};

struct Mesh {
  std::vector<triangle> tris;
  float offsetX, offsetY, offsetZ, rotatingMult, Yaw;
  Vector3 currentPos, ParentVec;
  Mat4x4 matRotX, matRotZ, matRotY;

  Mesh() {}

  Mesh(std::string Filename, Vector3 *parentVec, float offX, float offY,
       float offZ, float rotating) {
    if (parentVec)
      ParentVec = *parentVec;
    else
      ParentVec = {0.0f, 0.0f, 0.0f};
    offsetX = offX + ParentVec.x;
    offsetY = offY + ParentVec.y;
    offsetZ = offZ + ParentVec.z;
    if (rotating)
      rotatingMult = rotating;
    currentPos = {offsetX, offsetY, offsetZ};
    LoadFromObjectFile(Filename);
  }

  void UpdatePos(Vector3 *parentVec, float offX, float offY, float offZ) {
    if (parentVec)
      ParentVec = *parentVec;
    else
      ParentVec = {0, 0, 0};
    offsetX = ParentVec.x + offX;
    offsetY = ParentVec.y + offY;
    offsetZ = ParentVec.z + offZ;
    currentPos = {offsetX, offsetY, offsetZ};
  }

  bool LoadFromObjectFile(std::string sFilename) {
    std::ifstream f(sFilename);
    if (!f.is_open()) {
      return false;
    }

    std::vector<Vector3> verts;

    while (!f.eof()) {
      char line[128];
      f.getline(line, 128);
      std::strstream s;
      s << line;

      char junk;
      if (line[0] == 'v') {
        Vector3 v;
        s >> junk >> v.x >> v.y >> v.z;
        verts.push_back(v);
      }

      if (line[0] == 'f') {
        int f[3];
        s >> junk >> f[0] >> f[1] >> f[2];
        tris.push_back({verts[f[0] - 1], verts[f[1] - 1], verts[f[2] - 1]});
      }
    }

    return true;
  }
};

class GameObject {
public:
  Mesh objectMesh;
  float offsetX, offsetY, offsetZ;
  Vector3 currentPos, ParentVec;
  Mat4x4 matRotX, matRotZ, matRotY;

  GameObject();

  void UpdatePos(Vector3 *parentVec, float offX, float offY, float offZ);
  virtual void rotateShip(float angX, float angY, float angZ);
};

class SpaceShip : public GameObject {
public:
  SpaceShip(std::string objFile, Vector3 startPos,
            Vector3 parentVec = {0.0f, 0.0f, 0.0f});
  Mesh getMesh();
  Vector3 getPosition();
  void setMesh(std::string);
};

class Player : public GameObject {
public:
  float Speed;
  float maxSpeed;
  float Yaw, Pitch, Roll;
  Player();
};

class Scene {
public:
  GameObject GameObjects[15];
  virtual void Update(float) {};
  int SceneId;
};

class Rendering3D {
public:
  void MultiplyMatrixVector(Vector3 &i, Vector3 &o, Mat4x4 &m);
  Mat4x4 MakeMat();
  Mat4x4 MatPAt(Vector3 &pos, Vector3 &target, Vector3 &up);
  void DefineMesh();
  void DrawTriangleW(SDL_Renderer *, int, int, int, int, int, int);
  void DrawMesh(GameObject, SDL_Renderer *);
  void DefineRotMat(double thetaX, double thetaY, double thetaZ);
  void drawThickLine(SDL_Renderer *, float, float, float, float, float);
  Vector3 VectorAdd(Vector3 &v1, Vector3 &v2);
  Vector3 VectorSub(Vector3 &v1, Vector3 &v2);
  Vector3 VectorMult(Vector3 &v, float k);
  Vector3 VectorDiv(Vector3 &v, float k);
  float VectorDotProduct(Vector3 &v1, Vector3 &v2);
  float VectorLength(Vector3 &v);
  Vector3 VectorNorm(Vector3 &v);
  Vector3 VectorCrosProd(Vector3 &v1, Vector3 &v2);
  Vector3 IntersectPlane(Vector3 &plane_p, Vector3 &plane_n, Vector3 &lineStart,
                         Vector3 &lineEnd);
  int TriangleVClipPlane(Vector3 plane_p, Vector3 plane_n, triangle &in_tri,
                         triangle &out_tri1, triangle &out_tri2);
  Mat4x4 MatMakeRotationX(float);
  Mat4x4 MatMakeRotationY(float);
  Mat4x4 MatMakeRotationZ(float);
  Mat4x4 MatMakeTranslation(float, float, float);
  Mat4x4 MultiplyMatrix(Mat4x4 &m1, Mat4x4 &m2);
  Mat4x4 MatrixQuickInverse(Mat4x4 &m);
  Vector3 GetNormalVector3(Vector3 &v);
};

class standardPixelShader {
public:
  void DrawPixels(SDL_Renderer *);
};
