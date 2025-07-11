#include "program.h"

extern Rendering3D Rend3d;

GameObject::GameObject() {
  offsetX = 0;
  offsetY = 0;
  offsetZ = 0;
  currentPos = {offsetX, offsetY, offsetZ};
}

void GameObject::UpdatePos(Vector3 *parentVec, float offX, float offY,
                           float offZ) {
  if (parentVec)
    ParentVec = *parentVec;
  else
    ParentVec = {0, 0, 0};
  offsetX = ParentVec.x + offX;
  offsetY = ParentVec.y + offY;
  offsetZ = ParentVec.z + offZ;
  currentPos = {offsetX, offsetY, offsetZ};
}

void GameObject::rotateShip(float angX, float angY, float angZ) {
  matRotX = Rend3d.MatMakeRotationX(angX);
  matRotY = Rend3d.MatMakeRotationY(angY);
  matRotZ = Rend3d.MatMakeRotationZ(angZ);
}
