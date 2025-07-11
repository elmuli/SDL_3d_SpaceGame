#include "Scene.h"
#include "program.h"

#include <SDL3/SDL_main.h>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <ctime>
#include <iostream>
#include <iterator>
#include <list>
#include <ostream>

Rendering3D Rend3d;

Scene *scenes[10];

std::list<Mesh> meshes;
std::list<GameObject> GameObjects;
Mat4x4 matProj;
Mat4x4 matRotX;
Mat4x4 matRotZ;
Vector3 vCamera;
Vector3 vLookDir;
float Yaw, Pitch;
double ThetaDelta;

int currentScene;

Player player;

const int WIDTH = 800, HIGHT = 700;

void MovePlayer(float DeltaTime, GameObject obj[]) {

  Vector3 vForvard = Rend3d.VectorMult(vLookDir, player.Speed * DeltaTime);

  vCamera = Rend3d.VectorAdd(vCamera, vForvard);

  obj[14].matRotY = Rend3d.MatMakeRotationY(player.Yaw);
  obj[14].matRotX = Rend3d.MatMakeRotationX(player.Pitch);
  obj[14].matRotZ = Rend3d.MatMakeRotationZ(player.Roll);
  obj[14].UpdatePos(&vCamera, 0.0f, -0.25f, 0.0f);
}

void ScanKeys(const SDL_Event *e, float DeltaTime) {
  SDL_assert(e->type == SDL_EVENT_KEY_DOWN);
  Vector3 vForvardNorm = Rend3d.GetNormalVector3(vLookDir);
  Vector3 vSide = Rend3d.VectorMult(vForvardNorm, 8.0f * DeltaTime);
  if (e->key.scancode == SDL_SCANCODE_UP) {
    player.Pitch -= 2.0f * DeltaTime;
    // vCamera.y += 8.0f * DeltaTime;
  }
  if (e->key.scancode == SDL_SCANCODE_DOWN) {
    player.Pitch += 2.0f * DeltaTime;
    // vCamera.y -= 8.0f * DeltaTime;
  }
  if (e->key.scancode == SDL_SCANCODE_LEFT) {
    player.Yaw -= 2.0f * DeltaTime;
  }
  if (e->key.scancode == SDL_SCANCODE_RIGHT) {
    player.Yaw += 2.0f * DeltaTime;
  }
  if (e->key.scancode == SDL_SCANCODE_W) {
    if (player.Speed < player.maxSpeed)
      player.Speed += 1;
  }
  if (e->key.scancode == SDL_SCANCODE_S) {
    if (player.Speed > -player.maxSpeed)
      player.Speed -= 1;
  }
  if (e->key.scancode == SDL_SCANCODE_D) {
    player.Roll += 2.0f * DeltaTime;
  }
  if (e->key.scancode == SDL_SCANCODE_A) {
    player.Roll -= 2.0f * DeltaTime;
  }
  if (e->key.scancode == SDL_SCANCODE_E) {
    vCamera = Rend3d.VectorAdd(vCamera, vSide);
  }
  if (e->key.scancode == SDL_SCANCODE_Q) {
    vCamera = Rend3d.VectorSub(vCamera, vSide);
  }
}

Scene *SetUpScene(Scene *CurrentScene) {
  CurrentScene = scenes[currentScene];
  return CurrentScene;
}

GameObject *setObjects(GameObject objs[]) {
  for (int i = 0; i < 15; i++) {
    objs[i] = scenes[currentScene]->GameObjects[i];
  }
  return objs;
}

void CheckPos() {
  Vector3 def = {0, 0, 0};
  Vector3 Vpos = Rend3d.VectorSub(def, vCamera);
  float dist = Rend3d.VectorLength(Vpos);
  dist = std::abs(dist);
  if (dist > 200) {
    currentScene = 2;
  } else if (dist < 200) {
    currentScene = 1;
  }
}

int SDL_main(int argc, char *argv[]) {
  SDL_Init(SDL_INIT_VIDEO);

  SDL_SetHint("SDL_RENDER_LINE_METHOD", "3");

  SDL_Window *window = SDL_CreateWindow("test", WIDTH, HIGHT, 0);
  SDL_Renderer *renderer = SDL_CreateRenderer(window, NULL);
  SDL_SetRenderScale(renderer, 1, 1);

  auto start = std::chrono::steady_clock::now();
  auto start_ = start;
  scenes[1] = new Scene_1;
  scenes[2] = new Scene;
  Rend3d.DefineMesh();
  SDL_Event windowEvent;
  currentScene = 1;

  int cScene = currentScene;

  double frames = 0;
  double frameRate = 30;
  double averageFrameTime = 33.33;
  std::clock_t Time = 0;

  GameObject sceneObj[15];
  setObjects(sceneObj);

  while (true) {
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsedTime = end - start_;
    float DeltaTime = elapsedTime.count();

    if (SDL_PollEvent(&windowEvent)) {
      if (SDL_EVENT_QUIT == windowEvent.type) {
        break;
      }
      if (windowEvent.type == SDL_EVENT_KEY_DOWN) {
        ScanKeys(&windowEvent, DeltaTime);
      }
    }

    CheckPos();
    if (cScene != currentScene) {
      setObjects(sceneObj);
    }

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);

    ThetaDelta += (0.1 * DeltaTime);

    std::clock_t beginFrame = std::clock();

    MovePlayer(DeltaTime, sceneObj);

    scenes[cScene]->Update(ThetaDelta);

    for (int i = 0; i < 15; i++) {
      Rend3d.DrawMesh(sceneObj[i], renderer);
    }

    cScene = currentScene;

    std::clock_t endFrame = std::clock();
    Time += endFrame - beginFrame;
    frames++;

    if (((Time / (double)CLOCKS_PER_SEC) * 1000) > 100.0) { // every second
      frameRate = (double)frames * 0.5 + frameRate * 0.5;   // more stable
      frames = 0;
      Time -= CLOCKS_PER_SEC;
      averageFrameTime = 1000.0 / (frameRate == 0 ? 0.001 : frameRate);
    }

    SDL_SetRenderDrawColor(renderer, 51, 102, 255,
                           SDL_ALPHA_OPAQUE); /* light blue, full alpha */
    SDL_RenderDebugTextFormat(renderer, 50, 50, "ms/f: %lf", averageFrameTime,
                              SDL_GetTicks() / 100);
    SDL_RenderDebugTextFormat(renderer, 50, 80, "vCamera z: %lf", vCamera.z,
                              SDL_GetTicks() / 1000);

    SDL_Delay(16);
    SDL_RenderPresent(renderer);
    start_ = end;
  }

  for (int i = 0; i < 2; ++i) {
    delete scenes[i];
  }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return 0;
}
