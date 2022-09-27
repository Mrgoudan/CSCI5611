int numObstacles = 100;
int numNodes  = 100; 
  

static int maxNumObstacles = 1000;
Vec2 circlePos[] = new Vec2[maxNumObstacles]; //Circle positions
float circleRad[] = new float[maxNumObstacles];  //Circle radii

Vec2 startPos; //= new Vec2(100,500);
Vec2 goalPos;// = new Vec2(500,200);

static int maxNumNodes = 1000;
Vec2[] nodePos = new Vec2[maxNumNodes];


ArrayList<Integer> curPath;

int strokeWidth = 2;
//void setup(){
//  size(1024,768);
//  testPRM();
//}

int numCollisions;
float pathLength;
boolean reachedGoal;
float agentSpeed = 100;

//The agent states
Vec2 agentPos;
Vec2 agentVel;
Vec2 point1;
Vec2 point2;
float agentRad = 30;

// Vec2 agentAcc;

//The agent goals
//Vec2 goalPos;

ArrayList<Integer> path;

int len;
int i;



void setup(){
  size(850,650);
  //size(850,650,P3D); //Smoother
  surface.setTitle("2D simulation");
  //Set initial agent positions and goals

  
  goalPos = new Vec2(200,420);

  placeRandomObstacles(numObstacles);
  //agentVel = sampleFreePos();
  agentPos= sampleFreePos();
  goalPos = sampleFreePos();
  generateRandomNodes(numNodes, circlePos, circleRad);
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  path = planPath(agentPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);

  len = path.size();
  i = 0;
  //Set initial velocities to cary agents towards their goals
    // agentVel[i] = goalPos[i].minus(agentPos[i]);
    // agentVel[i].setToLength(goalSpeed);

}


void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii){
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
    //boolean insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    while (insideAnyCircle){
      randPos = new Vec2(random(width),random(height));
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
      //insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    }
    nodePos[i] = randPos;
  }
}

void placeRandomObstacles(int numObstacles){
  //Initial obstacle position
  for (int i = 0; i < numObstacles; i++){
    circlePos[i] = new Vec2(random(50,950),random(50,700));
    circleRad[i] = (10+40*pow(random(1),3));
  }
  circleRad[0] = 30; //Make the first obstacle big
}

//Update agent positions & velocities based acceleration
void moveAgent(float dt){
  //Compute accelerations for every agents
  int goalNode = path.get(i);
  Vec2 goal = new Vec2(nodePos[goalNode].x, nodePos[goalNode].y);
  Vec2 dir = goal.minus(agentPos);
  if(dir.length() < agentSpeed*dt){
    agentVel = new Vec2(0,0);
    agentPos = goal;
    if(i < len - 1) i++;
  }else{
    dir.normalize();
    agentVel = dir.times(agentSpeed*dt);
  }
  agentPos.add(agentVel);
}

boolean paused = true;
void draw(){
  background(255,255,255); //White background
  stroke(0,0,0);
  fill(255,255,255);
   for (int i = 0; i < numObstacles; i++){
    Vec2 c = circlePos[i];
    float r = circleRad[i];
    circle(c.x,c.y,r*2);
  }

  //Update agent if not paused
  if (!paused){
    moveAgent(1.0/frameRate);
  }
 
  //Draw orange goal rectangle
   fill(255,150,50);
   rect(goalPos.x, goalPos.y, 20, 20);

 
  //Draw the green agents
  fill(20,200,150);

  circle(agentPos.x, agentPos.y, (agentRad - 10)*2);  //No.4 smaller radius
   //Draw PRM Nodes
  fill(0);
  for (int i = 0; i < numNodes; i++){
     circle(nodePos[i].x,nodePos[i].y,5);
  }
  
  //Draw graph
  stroke(100,100,100);
  strokeWeight(1);
  for (int i = 0; i < numNodes; i++){
    for (int j : neighbors[i]){
      line(nodePos[i].x,nodePos[i].y,nodePos[j].x,nodePos[j].y);
    }
  }
}

//Pause/unpause the simulation
void keyPressed(){
  if (key == ' ') paused = !paused;
}

Vec2 sampleFreePos(){
  Vec2 randPos = new Vec2(random(width),random(height));
  boolean insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos,2);
  while (insideAnyCircle){
    randPos = new Vec2(random(width),random(height));
    insideAnyCircle = pointInCircleList(circlePos,circleRad,numObstacles,randPos,2);
  }
  return randPos;
}
///////////////////////

float rayCircleIntersectTime(Vec2 center, float r, Vec2 l_start, Vec2 l_dir){
 
  //Compute displacement vector pointing from the start of the line segment to the center of the circle
  Vec2 toCircle = center.minus(l_start);
 
  //Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
  float a = l_dir.length()*l_dir.length();
  float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
  float c = toCircle.lengthSqr() - (r*r); //different of squared distances
 
  float d = b*b - 4*a*c; //discriminant
 
  if (d >=0 ){
    //If d is positive we know the line is colliding
    float t = (-b - sqrt(d))/(2*a); //Optimization: we typically only need the first collision!
    if (t >= 0) return t;
    return -1;
  }
 
  return -1; //We are not colliding, so there is no good t to return
}
