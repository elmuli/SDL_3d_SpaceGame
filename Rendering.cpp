#include "program.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <list>
#include <vector>

extern std::list<GameObject> GameObjects;
extern std::list<Mesh> meshes;

extern Scene *scenes[];
extern int currentScene;

extern Rendering3D Rend3d;
extern standardPixelShader pixelShader;

extern Player player;

extern Vector3 vCamera;
extern Vector3 vLookDir;

extern Mat4x4 matRotX;
extern Mat4x4 matRotZ;
extern Mat4x4 matProj;
extern double ThetaDelta;

void Rendering3D::DefineMesh() {
  Vector3 corStart = {0.0f, 0.0f, 0.0f};
  SpaceShip ship("Ship.obj", corStart);

  ship.UpdatePos(&corStart, 0.0f, 0.0f, 8.0f);

  float fNear = 0.1f;
  float fFar = 1000.0f;
  float fFov = 85.0f;
  float fAspectRatio = 700.0 / 800.0;
  float fFovRad = 1.0f / tanf(fFov * 0.5f / 180.0f * 3.14159f);
  matProj.m[0][0] = fAspectRatio * fFovRad;
  matProj.m[1][1] = fFovRad;
  matProj.m[2][2] = fFar / (fFar - fNear);
  matProj.m[3][2] = (-fFar * fNear) / (fFar - fNear);
  matProj.m[2][3] = 1.0f;
  matProj.m[3][3] = 0.0f;
}

void Rendering3D::DrawTriangleW(SDL_Renderer *renderer, int x1, int y1, int x2,
                                int y2, int x3, int y3) {
  SDL_Vertex vert[3];
  vert[0].position.x = x1;
  vert[0].position.y = y1;
  vert[1].position.x = x2;
  vert[1].position.y = y2;
  vert[2].position.x = x3;
  vert[2].position.y = y3;
  for (int i = 0; i > 4; i++) {
    vert[i].color.r = 0;
    vert[i].color.g = 0;
    vert[i].color.b = 255;
    vert[i].color.a = 255;
  }
  SDL_RenderGeometry(renderer, NULL, vert, 3, NULL, 0);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
  SDL_RenderLine(renderer, x1, y1, x2, y2);
  SDL_RenderLine(renderer, x2, y2, x3, y3);
  SDL_RenderLine(renderer, x3, y3, x1, y1);
}

void Rendering3D::DrawMesh(GameObject gameObj, SDL_Renderer *renderer) {

  Mat4x4 matTrans;
  matTrans = Rend3d.MatMakeTranslation(0.0f, 0.0f, 0.0f);

  Mat4x4 matWorld;
  matWorld = Rend3d.MakeMat();
  // std::cout << mesh.matRotX.m[2][2] << std::endl;
  matWorld = Rend3d.MultiplyMatrix(gameObj.matRotY, gameObj.matRotX);
  matWorld = Rend3d.MultiplyMatrix(matWorld, gameObj.matRotZ);
  matWorld = Rend3d.MultiplyMatrix(matWorld, matTrans);

  Vector3 vUp = {0, 1, 0};
  Vector3 vTarget = {0, 0, 1};
  Mat4x4 matCameraRotY = Rend3d.MatMakeRotationY(player.Yaw);
  Mat4x4 matCameraRotX = Rend3d.MatMakeRotationX(player.Pitch);
  Mat4x4 matCameraRotZ = Rend3d.MatMakeRotationZ(player.Roll);
  Mat4x4 matCameraRot = Rend3d.MultiplyMatrix(matCameraRotY, matCameraRotX);
  matCameraRot = Rend3d.MultiplyMatrix(matCameraRot, matCameraRotZ);
  Rend3d.MultiplyMatrixVector(vTarget, vLookDir, matCameraRot);
  vTarget = Rend3d.VectorAdd(vCamera, vLookDir);
  Mat4x4 matCamera = Rend3d.MatPAt(vCamera, vTarget, vUp);

  Mat4x4 matView = Rend3d.MatrixQuickInverse(matCamera);

  std::vector<triangle> vecTrianglesToRaster;

  for (auto tri : gameObj.objectMesh.tris) {
    // std::cout << "tri" << std::endl;
    triangle triProjected, triTransformed, triViewed;

    Rend3d.MultiplyMatrixVector(tri.p[0], triTransformed.p[0], matWorld);
    Rend3d.MultiplyMatrixVector(tri.p[1], triTransformed.p[1], matWorld);
    Rend3d.MultiplyMatrixVector(tri.p[2], triTransformed.p[2], matWorld);

    triTransformed.p[0].x += gameObj.offsetX;
    triTransformed.p[0].y += gameObj.offsetY;
    triTransformed.p[0].z += gameObj.offsetZ;
    triTransformed.p[1].x += gameObj.offsetX;
    triTransformed.p[1].y += gameObj.offsetY;
    triTransformed.p[1].z += gameObj.offsetZ;
    triTransformed.p[2].x += gameObj.offsetX;
    triTransformed.p[2].y += gameObj.offsetY;
    triTransformed.p[2].z += gameObj.offsetZ;

    Vector3 normal;
    Vector3 line1;
    Vector3 line2;

    line1 = Rend3d.VectorSub(triTransformed.p[1], triTransformed.p[0]);
    line2 = Rend3d.VectorSub(triTransformed.p[2], triTransformed.p[0]);

    normal = Rend3d.VectorCrosProd(line1, line2);
    normal = Rend3d.VectorNorm(normal);

    Vector3 vCameraRay = VectorSub(triTransformed.p[0], vCamera);
    if (Rend3d.VectorLength(vCameraRay) <= 1000) {
      if (Rend3d.VectorDotProduct(normal, vCameraRay) < 0.0f) {
        // std::cout << "Viewing" << std::endl;
        Rend3d.MultiplyMatrixVector(triTransformed.p[0], triViewed.p[0],
                                    matView);
        Rend3d.MultiplyMatrixVector(triTransformed.p[1], triViewed.p[1],
                                    matView);
        Rend3d.MultiplyMatrixVector(triTransformed.p[2], triViewed.p[2],
                                    matView);

        int clippedTriangles = 0;
        triangle clipped[2];
        clippedTriangles =
            Rend3d.TriangleVClipPlane({0.0f, 0.0f, 0.1f}, {0.0f, 0.0f, 1.0f},
                                      triViewed, clipped[0], clipped[1]);
        for (int n = 0; n < clippedTriangles; n++) {
          Rend3d.MultiplyMatrixVector(clipped[n].p[0], triProjected.p[0],
                                      matProj);
          Rend3d.MultiplyMatrixVector(clipped[n].p[1], triProjected.p[1],
                                      matProj);
          Rend3d.MultiplyMatrixVector(clipped[n].p[2], triProjected.p[2],
                                      matProj);

          triProjected.p[0].x *= -1.0f;
          triProjected.p[1].x *= -1.0f;
          triProjected.p[2].x *= -1.0f;
          triProjected.p[0].y *= -1.0f;
          triProjected.p[1].y *= -1.0f;
          triProjected.p[2].y *= -1.0f;

          Vector3 vOffsetView = {1, 1, 0};
          triProjected.p[0] = VectorAdd(triProjected.p[0], vOffsetView);
          triProjected.p[1] = VectorAdd(triProjected.p[1], vOffsetView);
          triProjected.p[2] = VectorAdd(triProjected.p[2], vOffsetView);
          triProjected.p[0].x *= 0.5f * 800;
          triProjected.p[0].y *= 0.5f * 700;
          triProjected.p[1].x *= 0.5f * 800;
          triProjected.p[1].y *= 0.5f * 700;
          triProjected.p[2].x *= 0.5f * 800;
          triProjected.p[2].y *= 0.5f * 700;

          vecTrianglesToRaster.push_back(triProjected);
        }
      }
    }
  }

  std::sort(vecTrianglesToRaster.begin(), vecTrianglesToRaster.end(),
            [](triangle &t1, triangle &t2) {
              float z1 = (t1.p[0].z + t1.p[1].z + t1.p[2].z) / 3.0f;
              float z2 = (t2.p[0].z + t2.p[1].z + t2.p[2].z) / 3.0f;
              return z1 > z2;
            });

  for (auto &triProjected : vecTrianglesToRaster) {
    Rend3d.DrawTriangleW(renderer, triProjected.p[0].x, triProjected.p[0].y,
                         triProjected.p[1].x, triProjected.p[1].y,
                         triProjected.p[2].x, triProjected.p[2].y);
  }
}

void Rendering3D::MultiplyMatrixVector(Vector3 &i, Vector3 &o, Mat4x4 &m) {
  o.x = i.x * m.m[0][0] + i.y * m.m[1][0] + i.z * m.m[2][0] + m.m[3][0];
  o.y = i.x * m.m[0][1] + i.y * m.m[1][1] + i.z * m.m[2][1] + m.m[3][1];
  o.z = i.x * m.m[0][2] + i.y * m.m[1][2] + i.z * m.m[2][2] + m.m[3][2];
  float w = i.x * m.m[0][3] + i.y * m.m[1][3] + i.z * m.m[2][3] + m.m[3][3];

  if (w != 0.0f) {
    o.x /= w;
    o.y /= w;
    o.z /= w;
  }
}

Mat4x4 Rendering3D::MultiplyMatrix(Mat4x4 &m1, Mat4x4 &m2) {
  Mat4x4 matrix;
  for (int c = 0; c < 4; c++)
    for (int r = 0; r < 4; r++)
      matrix.m[r][c] = m1.m[r][0] * m2.m[0][c] + m1.m[r][1] * m2.m[1][c] +
                       m1.m[r][2] * m2.m[2][c] + m1.m[r][3] * m2.m[3][c];
  return matrix;
}

void Rendering3D::DefineRotMat(double thetaX, double thetaY, double thetaZ) {
  matRotZ.m[0][0] = cos(thetaZ);
  matRotZ.m[0][1] = sin(thetaZ);
  matRotZ.m[1][0] = -sin(thetaZ);
  matRotZ.m[1][1] = cos(thetaZ);
  matRotZ.m[2][2] = 1;
  matRotZ.m[3][3] = 1;

  matRotX.m[0][0] = 1;
  matRotX.m[1][1] = cos(thetaX);
  matRotX.m[1][2] = sin(thetaX);
  matRotX.m[2][1] = -sin(thetaX);
  matRotX.m[2][2] = cos(thetaX);
  matRotX.m[3][3] = 1;
  /*
    matRotY.m[0][0] = cos(thetaY);
    matRotY.m[0][2] = sin(thetaY);
    matRotY.m[1][1] = 1;
    matRotY.m[2][0] = -sin(thetaY);
    matRotY.m[2][2] = cos(thetaY);
    matRotY.m[3][3] = 1;*/
}

Mat4x4 Rendering3D::MakeMat() {
  Mat4x4 matrix;
  matrix.m[0][0] = 1.0f;
  matrix.m[1][1] = 1.0f;
  matrix.m[2][2] = 1.0f;
  matrix.m[3][3] = 1.0f;
  return matrix;
}

Mat4x4 Rendering3D::MatMakeRotationX(float fAngleRad) {
  Mat4x4 matrix;
  matrix.m[0][0] = 1.0f;
  matrix.m[1][1] = cos(fAngleRad);
  matrix.m[1][2] = sin(fAngleRad);
  matrix.m[2][1] = -sin(fAngleRad);
  matrix.m[2][2] = cos(fAngleRad);
  matrix.m[3][3] = 1.0f;
  return matrix;
}

Mat4x4 Rendering3D::MatMakeRotationY(float fAngleRad) {
  Mat4x4 matrix;
  matrix.m[0][0] = cos(fAngleRad);
  matrix.m[0][2] = sin(fAngleRad);
  matrix.m[2][0] = -sin(fAngleRad);
  matrix.m[1][1] = 1.0f;
  matrix.m[2][2] = cos(fAngleRad);
  matrix.m[3][3] = 1.0f;
  return matrix;
}

Mat4x4 Rendering3D::MatMakeRotationZ(float fAngleRad) {
  Mat4x4 matrix;
  matrix.m[0][0] = cos(fAngleRad);
  matrix.m[0][1] = sin(fAngleRad);
  matrix.m[1][0] = -sin(fAngleRad);
  matrix.m[1][1] = cos(fAngleRad);
  matrix.m[2][2] = 1.0f;
  matrix.m[3][3] = 1.0f;
  return matrix;
}

Mat4x4 Rendering3D::MatMakeTranslation(float x, float y, float z) {
  Mat4x4 matrix;
  matrix.m[0][0] = 1.0f;
  matrix.m[1][1] = 1.0f;
  matrix.m[2][2] = 1.0f;
  matrix.m[3][3] = 1.0f;
  matrix.m[3][0] = x;
  matrix.m[3][1] = y;
  matrix.m[3][2] = z;
  return matrix;
}

Mat4x4 Rendering3D::MatrixQuickInverse(Mat4x4 &m) {
  Mat4x4 matrix;
  matrix.m[0][0] = m.m[0][0];
  matrix.m[0][1] = m.m[1][0];
  matrix.m[0][2] = m.m[2][0];
  matrix.m[0][3] = 0.0f;
  matrix.m[1][0] = m.m[0][1];
  matrix.m[1][1] = m.m[1][1];
  matrix.m[1][2] = m.m[2][1];
  matrix.m[1][3] = 0.0f;
  matrix.m[2][0] = m.m[0][2];
  matrix.m[2][1] = m.m[1][2];
  matrix.m[2][2] = m.m[2][2];
  matrix.m[2][3] = 0.0f;
  matrix.m[3][0] = -(m.m[3][0] * matrix.m[0][0] + m.m[3][1] * matrix.m[1][0] +
                     m.m[3][2] * matrix.m[2][0]);
  matrix.m[3][1] = -(m.m[3][0] * matrix.m[0][1] + m.m[3][1] * matrix.m[1][1] +
                     m.m[3][2] * matrix.m[2][1]);
  matrix.m[3][2] = -(m.m[3][0] * matrix.m[0][2] + m.m[3][1] * matrix.m[1][2] +
                     m.m[3][2] * matrix.m[2][2]);
  matrix.m[3][3] = 1.0f;
  return matrix;
}

Mat4x4 Rendering3D::MatPAt(Vector3 &pos, Vector3 &target, Vector3 &up) {
  Vector3 newForward = VectorSub(target, pos);
  newForward = VectorNorm(newForward);

  Vector3 a = VectorMult(newForward, VectorDotProduct(up, newForward));
  Vector3 newUp = VectorSub(up, a);
  newUp = VectorNorm(newUp);

  Vector3 newRight = VectorCrosProd(newUp, newForward);

  Mat4x4 matrix;
  matrix.m[0][0] = newRight.x;
  matrix.m[0][1] = newRight.y;
  matrix.m[0][2] = newRight.z;
  matrix.m[0][3] = 0.0f;
  matrix.m[1][0] = newUp.x;
  matrix.m[1][1] = newUp.y;
  matrix.m[1][2] = newUp.z;
  matrix.m[1][3] = 0.0f;
  matrix.m[2][0] = newForward.x;
  matrix.m[2][1] = newForward.y;
  matrix.m[2][2] = newForward.z;
  matrix.m[2][3] = 0.0f;
  matrix.m[3][0] = pos.x;
  matrix.m[3][1] = pos.y;
  matrix.m[3][2] = pos.z;
  matrix.m[3][3] = 1.0f;
  return matrix;
}

Vector3 Rendering3D::VectorAdd(Vector3 &v1, Vector3 &v2) {
  return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
}

Vector3 Rendering3D::VectorSub(Vector3 &v1, Vector3 &v2) {
  return {v1.x - v2.x, v1.y - v2.y, v1.z - v2.z};
}

Vector3 Rendering3D::VectorMult(Vector3 &v1, float k) {
  return {v1.x * k, v1.y * k, v1.z * k};
}

Vector3 Rendering3D::VectorDiv(Vector3 &v1, float k) {
  return {v1.x / k, v1.y / k, v1.z / k};
}

float Rendering3D::VectorDotProduct(Vector3 &v1, Vector3 &v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

float Rendering3D::VectorLength(Vector3 &v) {
  return sqrt(VectorDotProduct(v, v));
}

Vector3 Rendering3D::VectorNorm(Vector3 &v) {
  float l = VectorLength(v);
  return {v.x / l, v.y / l, v.z / l};
}

Vector3 Rendering3D::VectorCrosProd(Vector3 &v1, Vector3 &v2) {
  Vector3 v;
  v.x = v1.y * v2.z - v1.z * v2.y;
  v.y = v1.z * v2.x - v1.x * v2.z;
  v.z = v1.x * v2.y - v1.y * v2.x;
  return v;
}

Vector3 Rendering3D::GetNormalVector3(Vector3 &v) {
  Vector3 vUnitVec = Rend3d.VectorNorm(v);
  Vector3 vUp = {0.0f, 1.0f, 0.0f};
  if (fabs(v.y > 0.9999f))
    vUp = {1.0f, 0.0f, 0.0f};
  return Rend3d.VectorCrosProd(vUnitVec, vUp);
}

Vector3 Rendering3D::IntersectPlane(Vector3 &plane_p, Vector3 &plane_n,
                                    Vector3 &lineStart, Vector3 &lineEnd) {
  plane_n = VectorNorm(plane_n);
  float plane_d = -VectorDotProduct(plane_n, plane_p);
  float ad = VectorDotProduct(lineStart, plane_n);
  float bd = VectorDotProduct(lineEnd, plane_n);
  float t = (-plane_d - ad) / (bd - ad);
  Vector3 lineStartToEnd = VectorSub(lineEnd, lineStart);
  Vector3 lineToIntersect = VectorMult(lineStartToEnd, t);
  return VectorAdd(lineStart, lineToIntersect);
}

int Rendering3D::TriangleVClipPlane(Vector3 plane_p, Vector3 plane_n,
                                    triangle &in_tri, triangle &out_tri1,
                                    triangle &out_tri2) {
  plane_n = VectorNorm(plane_n);

  auto dist = [&](Vector3 &p) {
    Vector3 n = VectorNorm(p);
    return (plane_n.x * p.x + plane_n.y * p.y + plane_n.z * p.z -
            VectorDotProduct(plane_n, plane_p));
  };

  Vector3 *inside_points[3];
  int nInsidePointCount = 0;
  Vector3 *outside_points[3];
  int nOutsidePointCount = 0;

  float d0 = dist(in_tri.p[0]);
  float d1 = dist(in_tri.p[1]);
  float d2 = dist(in_tri.p[2]);

  if (d0 >= 0) {
    inside_points[nInsidePointCount++] = &in_tri.p[0];
  } else {
    outside_points[nOutsidePointCount++] = &in_tri.p[0];
  }
  if (d1 >= 0) {
    inside_points[nInsidePointCount++] = &in_tri.p[1];
  } else {
    outside_points[nOutsidePointCount++] = &in_tri.p[1];
  }
  if (d2 >= 0) {
    inside_points[nInsidePointCount++] = &in_tri.p[2];
  } else {
    outside_points[nOutsidePointCount++] = &in_tri.p[2];
  }

  if (nInsidePointCount == 0) {
    return 0;
  }

  if (nInsidePointCount == 3) {
    out_tri1 = in_tri;

    return 1;
  }

  if (nInsidePointCount == 1 && nOutsidePointCount == 2) {

    out_tri1.p[0] = *inside_points[0];

    out_tri1.p[1] =
        IntersectPlane(plane_p, plane_n, *inside_points[0], *outside_points[0]);
    out_tri1.p[2] =
        IntersectPlane(plane_p, plane_n, *inside_points[0], *outside_points[1]);

    return 1;
  }

  if (nInsidePointCount == 2 && nOutsidePointCount == 1) {
    out_tri1.p[0] = *inside_points[0];
    out_tri1.p[1] = *inside_points[1];
    out_tri1.p[2] =
        IntersectPlane(plane_p, plane_n, *inside_points[0], *outside_points[0]);

    out_tri2.p[0] = *inside_points[1];
    out_tri2.p[1] = out_tri1.p[2];
    out_tri2.p[2] =
        IntersectPlane(plane_p, plane_n, *inside_points[1], *outside_points[0]);

    return 2;
  }
}
