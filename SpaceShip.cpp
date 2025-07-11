#include "program.h"
#include <string>

extern std::list<Mesh> meshes;
extern Rendering3D Rend3d;

SpaceShip::SpaceShip(std::string objFile, Vector3 startPos, Vector3 parentVec) {
  setMesh(objFile);
  ParentVec = parentVec;
  offsetX = startPos.x;
  offsetY = startPos.y;
  offsetZ = startPos.z;
  currentPos = {offsetX, offsetY, offsetZ};
}

Vector3 SpaceShip::getPosition() { return currentPos; }
Mesh SpaceShip::getMesh() { return objectMesh; }
void SpaceShip::setMesh(std::string obj) { objectMesh.LoadFromObjectFile(obj); }
