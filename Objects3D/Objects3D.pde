// Created for CSCI 5611 by Dan Shervheim
// Monkey mesh is the default Blender starting file.
// Tiger mesh is by Jeremie Louvetz: https://sketchfab.com/3d-models/sumatran-tiger-95c4008c4c764c078f679d4c320e7b18#download

Camera camera;

ObjMesh monkeyMesh;
ObjMesh tigerMesh;

final int COUNT = 100;
PVector[] randomPositions = new PVector[COUNT];
PVector[] randomRotations = new PVector[COUNT];
float[] randomScales = new float[COUNT];

void setup()
{
  size(600, 600, P3D);
  camera = new Camera();
  monkeyMesh = new ObjMesh("monkey.obj");
  
  tigerMesh = new ObjMesh("tigre_sumatra_sketchfab.obj");
  tigerMesh.position = new PVector(0,0,-5.0);
  tigerMesh.rotation = new PVector(0, 0, 180);  // OBJ coordinate system y is reversed from processing, so rotate 180 to flip it.
  tigerMesh.scale = 2.5;
  
  for (int i = 0; i < COUNT; i++) {
    randomPositions[i] = new PVector(random(-10.0, 10.0), random(-10.0, 10.0), random(-10.0, 10.0) - 5.0);
    randomRotations[i] = new PVector(random(0, 360.0), random(0, 360.0), random(0, 360.0));
    randomScales[i] = random(0.1, 1.0);
  }
}

void keyPressed()
{
  camera.HandleKeyPressed();
}

void keyReleased()
{
  camera.HandleKeyReleased();
}

void draw()
{
  background(255);
  camera.Update(1.0/frameRate);
  
  directionalLight(255.0, 255.0, 255.0, -1, 1, -1);

  // Draws the tiger mesh at its given position.
  tigerMesh.draw();
  
  // Draw the monkey mesh at multiple different orientations in the scene.
  // This is useful to draw large amounts of the same object without having to load multiple copies in memory.
  monkeyMesh.drawInstanced(randomPositions, randomRotations, randomScales);
}
