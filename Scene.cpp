#include "Scene.h"
#include "program.h"
#include <iostream>

extern Rendering3D Rend3d;

Vector3 corStart = {0, 0, 0};

Scene_1::Scene_1() {
  SpaceShip cocpit("Cocpit.obj", corStart);
  SpaceShip speeder("Star_Speeder.obj", corStart);
  GameObject map;

  map.objectMesh.LoadFromObjectFile("map_test.obj");
  map.offsetY = -6.0f;
  map.matRotY = Rend3d.MatMakeRotationY(0.0f);
  map.matRotX = Rend3d.MatMakeRotationX(0.0f);
  map.matRotZ = Rend3d.MatMakeRotationZ(0.0f);

  speeder.UpdatePos(&corStart, 0.0f, 0.0f, 8.0f);

  cocpit.matRotY = Rend3d.MatMakeRotationY(0.0f);
  cocpit.matRotX = Rend3d.MatMakeRotationX(0.0f);
  cocpit.matRotZ = Rend3d.MatMakeRotationZ(0.0f);
  speeder.matRotY = Rend3d.MatMakeRotationY(0.0f);
  speeder.matRotX = Rend3d.MatMakeRotationX(0.0f);
  speeder.matRotZ = Rend3d.MatMakeRotationZ(90.0f);

  GameObjects[0] = map;
  GameObjects[1] = speeder;
  GameObjects[14] = cocpit;
}

void Scene_1::Update(float ThetaDelta) {
  GameObjects[1].matRotX = Rend3d.MatMakeRotationX(ThetaDelta);
}
