int numObstacles = 50;
int numNodes  = 100; 
  

static int maxNumObstacles = 1000;
Vec2 circlePos[] = new Vec2[maxNumObstacles]; //Circle positions
float circleRad[] = new float[maxNumObstacles];  //Circle radii

Vec2 startPos; //= new Vec2(100,500);
Vec2 goalPos;// = new Vec2(500,200);

static int maxNumNodes = 1000;
Vec2[] nodePos = new Vec2[maxNumNodes];


ArrayList<Vec2[]> path;
ArrayList<Integer> nodes;
int strokeWidth = 2;

int numCollisions;
float pathLength;
boolean reachedGoal;
// float agentSpeed = 100;

Vec2[] startingPos = new Vec2[maxNumNodes];
Vec2[] firstPos = new Vec2[maxNumNodes];
Vec2[] goalPosition = new Vec2[maxNumNodes];
int numOfAgents = 0;
Vec2 agentPos;
// Vec2 agentVel;
float agentRad = 10;

int[] posPlace = new int[maxNumNodes];

float k_goal = 5;  //TODO: Tune this parameter to agent stop naturally on their goals
float k_avoid = 20;
float goalSpeed = 5;

// Vec2[] agentPos = new Vec2[maxNumNodes];
Vec2[] agentVel = new Vec2[maxNumNodes];
Vec2[] agentAcc = new Vec2[maxNumNodes];
PImage huamn;
PImage blackhole;
PImage space;
PImage aliean;
PImage spaceship;
int len;



// Vec2 agentAcc;
//The agent states
//The agent goals
//Vec2 goalPos;
//void setup(){
//  size(1024,768);
//  testPRM();
//}

void setup(){
  size(850,650);
  //size(850,650,P3D); //Smoother
  huamn = loadImage("human.png");
  blackhole = loadImage("blackhole.png");
  space = loadImage("space.jpeg");
  aliean = loadImage("aliean.png");
  spaceship = loadImage("spaceship.png");

  surface.setTitle("2D simulation");
  runSimulation();
  //Set initial agent positions and goals
}
void runSimulation(){
  goalPos = new Vec2(200,420);

  placeRandomObstacles(numObstacles);
  agentPos= sampleFreePos();
  startPos = new Vec2(agentPos.x, agentPos.y);
  startingPos[0]=startPos;
  firstPos[0] = startPos;
  numOfAgents= numOfAgents+1;
  goalPos = sampleFreePos();
  goalPosition[0] = goalPos;
  generateRandomNodes(numNodes, circlePos, circleRad);
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  path = new ArrayList(numNodes);
  for(int j = 0;j<numOfAgents;j++){
    nodes = planPath(startingPos[j], goalPosition[j], circlePos, circleRad, numObstacles, nodePos, numNodes);
    println(nodes);
    Vec2 [] temp =  new Vec2[nodes.size()+1];
    path.add(temp);
    for(int z  = 0;z<nodes.size();z++){
      // println(nodePos[z-1].x, nodePos[z-1].y,nodePos[z].x, nodePos[z].y,nodePos[z+1].x, nodePos[z+1].y);
    path.get(j)[z]=new Vec2(nodePos[nodes.get(z)].x, nodePos[nodes.get(z)].y);
    
    }
    agentVel[j] = goalPosition[j].minus(startingPos[j]);
    if (agentVel[j].length() > 0)
      agentVel[j].setToLength(goalSpeed);
    path.get(j)[nodes.size()]=goalPosition[j];
    println(path.get(j));
    len = path.get(j).length;
    posPlace[j] = 0;
  }
}
  
  //Set initial velocities to cary agents towards their goals
    // agentVel[i] = goalPos[i].minus(agentPos[i]);
    // agentVel[i].setToLength(goalSpeed);

// void restart(){

// }

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
    circleRad[i] = (20+40*pow(random(1),3));
  }
  circleRad[0] = 30; //Make the first obstacle big
}

//Update agent positions & velocities based acceleration
void moveAgent(float dt){
  //Compute accelerations for every agents
  for(int j = 0;j<numOfAgents;j++){
    Vec2 next = path.get(j)[posPlace[j]];
    Vec2 dir;

    dir = next.minus(startingPos[j]);
    
    //shortcut design
    if(posPlace[j] < path.get(j).length - 1){

      Vec2 node2 = path.get(j)[posPlace[j]+1];
      Vec2 nextdir = node2.minus(startingPos[j]).normalized();
      float dist = node2.distanceTo(startingPos[j]);
      hitInfo shortcut = rayCircleListIntersect(circlePos, circleRad, numObstacles, startingPos[j], nextdir, dist);
      if(!shortcut.hit){
        posPlace[j] = posPlace[j]+1;
        print("used shortcut");
        return;
      }    
    }
    println("POSPLACE",posPlace[j]);
    if(startingPos[j].x == goalPosition[j].x &&startingPos[j].y == goalPosition[j].y ){
      continue;
    }
      print("in");
      agentAcc[j] = computeAgentForces(j);

      if(dir.length() < goalSpeed*dt){
        // agentVel = new Vec2(0,0);
        startingPos[j] = next;
        if(posPlace[j] < path.get(j).length - 1) posPlace[j] = posPlace[j]+1;
      }else{
        dir.normalize();
        // agentVel[j] = dir.times(goalSpeed*dt);
        agentVel[j].add(agentAcc[j].times(dt));
        
      }
    if(abs(startingPos[j].x+agentVel[j].x-goalPosition[j].x)<3 && abs(startingPos[j].y+agentVel[j].y-goalPosition[j].y)<3){
      startingPos[j] = goalPosition[j];
    }
    else{
    startingPos[j].add(agentVel[j]);}
    

  }
  // for (int i = 0; i < numOfAgents; i++){
  //    
  //   }
  //   //Update position and velocity using (Eulerian) numerical integration
  //   for (int i = 0; i < numOfAgents; i++){
  //     agentVel[i].add(agentAcc[i].times(dt));
  //     startingPos[i].add(agentVel[i].times(dt));
  //   }

}

boolean paused = true;
void draw(){
  background(255,255,255); //White background
  image(space,0,0,width,height);
  stroke(0,0,0);
  strokeWeight(1);
  fill(255,255,255);
  for (int i = 1; i < numObstacles; i++){
    Vec2 c = circlePos[i];
    float r = circleRad[i] - agentRad;
    circle(c.x,c.y,r*2);
    image(blackhole, c.x-r, c.y-r, r*2, r*2);

  }
  strokeWeight(1);
  fill(0,0,255);
  Vec2 c = circlePos[0];
    float r = circleRad[0] - agentRad;
    circle(c.x,c.y,r*2);
    image(huamn, c.x-r, c.y-r, r*2, r*2);

    
  //Update agent if not paused
  if (!paused){
    moveAgent(1.0/frameRate);
  }
 
  //Draw orange goal rectangle


 
  //Draw the green agents

  for(int i = 0;i<numOfAgents;i++){
    fill(255,150,50);
    println(goalPosition[i].x,goalPosition[i].y,goalPos.x,goalPos.y,startingPos[i].x,startingPos[i].y);
    rect(goalPosition[i].x-10, goalPosition[i].y-10, 20, 20);
    image(spaceship,goalPosition[i].x-10, goalPosition[i].y-10, 20, 20);
    fill(20,200,150);    
    circle(startingPos[i].x, startingPos[i].y, agentRad*2); 
    image(aliean,startingPos[i].x-agentRad, startingPos[i].y-agentRad, agentRad*2,agentRad*2);
  }
   //No.4 smaller radius
   //Draw PRM Nodes
  // fill(0);

  // for (int i = 0; i < numNodes; i++){
  //    circle(nodePos[i].x,nodePos[i].y,5);
  // }
  
  // //Draw graph
  // stroke(100,100,100);
  // strokeWeight(1);
  // for (int i = 0; i < numNodes; i++){
  //   for (int j : neighbors[i]){
  //     line(nodePos[i].x,nodePos[i].y,nodePos[j].x,nodePos[j].y);
  //   }
  // }

  // stroke(20,255,40);
  // strokeWeight(5);
  // for(int i = 0;i<numOfAgents;i++){
  // if (path.get(i).length == 0){
    

  //     line(firstPos[i].x, firstPos[i].y,goalPos.x,goalPos.y);
  //   return;
  // }
  
  // line(firstPos[i].x, firstPos[i].y,path.get(i)[0].x,path.get(i)[0].y);
      
  // for (int j = 0; j < path.get(i).length-1; j++){
  //   // int curNode = path.get(i)[j];
  //   // int nextNode = path.get(i)[j+1];
  //   // println(path.get(i)[j].x,path.get(i)[j].y,path.get(i)[j+1].x,path.get(i)[j+1].y);
  //   line(path.get(i)[j].x,path.get(i)[j].y,path.get(i)[j+1].x,path.get(i)[j+1].y);
  // }
  // line(goalPos.x,goalPos.y,path.get(i)[path.get(i).length-1].x,path.get(i)[path.get(i).length-1].y);
  // }
  
}

//Pause/unpause the simulation
void keyPressed(){
  if (key == ' ') {
    paused = !paused;
  }
  if (key == 'r'){
    nodePos = new Vec2[maxNumNodes];
    startingPos = new Vec2[maxNumNodes];
    firstPos = new Vec2[maxNumNodes];
    numOfAgents = 0;
    posPlace = new int[maxNumNodes];
    agentVel = new Vec2[maxNumNodes];
    agentAcc = new Vec2[maxNumNodes];
    runSimulation();
  }
  float speed = 15;

  if (keyCode == RIGHT){
    circlePos[0].x += speed;
    path = new ArrayList<Vec2[]>();
        posPlace = new int[maxNumNodes];



  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  // path = new ArrayList(numNodes);
  for(int j = 0;j<numOfAgents;j++){
    nodes = planPath(startingPos[j], goalPosition[j], circlePos, circleRad, numObstacles, nodePos, numNodes);
    println(nodes);
    Vec2 [] temp =  new Vec2[nodes.size()];
    path.add(temp);
    for(int z  = 0;z<nodes.size();z++){
      // println(nodePos[z-1].x, nodePos[z-1].y,nodePos[z].x, nodePos[z].y,nodePos[z+1].x, nodePos[z+1].y);
      println(path.get(j),nodePos[nodes.get(z)]);
    path.get(j)[z]=new Vec2(nodePos[nodes.get(z)].x, nodePos[nodes.get(z)].y);
    
    }
    agentVel[j] = goalPosition[j].minus(startingPos[j]);
    if (agentVel[j].length() > 0)
      agentVel[j].setToLength(goalSpeed);
    path.get(j)[nodes.size()-1]=goalPosition[j];
    println(path.get(j));
    len = path.get(j).length;
    posPlace[j] = 0;
  }
  }
  if (keyCode == LEFT){
    path = new ArrayList<Vec2[]>();
        posPlace = new int[maxNumNodes];

    circlePos[0].x -= speed;
      connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  // path = new ArrayList(numNodes);
  for(int j = 0;j<numOfAgents;j++){
    nodes = planPath(startingPos[j], goalPosition[j], circlePos, circleRad, numObstacles, nodePos, numNodes);
    println(nodes);
    Vec2 [] temp =  new Vec2[nodes.size()];
    path.add(temp);
    for(int z  = 0;z<nodes.size();z++){
      // println(nodePos[z-1].x, nodePos[z-1].y,nodePos[z].x, nodePos[z].y,nodePos[z+1].x, nodePos[z+1].y);
      println(path.get(j),nodePos[nodes.get(z)]);
    path.get(j)[z]=new Vec2(nodePos[nodes.get(z)].x, nodePos[nodes.get(z)].y);
    
     }
    agentVel[j] = goalPosition[j].minus(startingPos[j]);
    if (agentVel[j].length() > 0)
      agentVel[j].setToLength(goalSpeed);
    path.get(j)[nodes.size()-1]=goalPosition[j];
    println(path.get(j));
    len = path.get(j).length;
    posPlace[j] = 0;
  }
  }
  if (keyCode == UP){
    path = new ArrayList<Vec2[]>();
        posPlace = new int[maxNumNodes];

    circlePos[0].y -= speed;
      connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  // path = new ArrayList(numNodes);
  for(int j = 0;j<numOfAgents;j++){
    nodes = planPath(startingPos[j], goalPosition[j], circlePos, circleRad, numObstacles, nodePos, numNodes);
    println(nodes);
    Vec2 [] temp =  new Vec2[nodes.size()];
    path.add(temp);
    for(int z  = 0;z<nodes.size();z++){
      // println(nodePos[z-1].x, nodePos[z-1].y,nodePos[z].x, nodePos[z].y,nodePos[z+1].x, nodePos[z+1].y);
      println(path.get(j),nodePos[nodes.get(z)]);
    path.get(j)[z]=new Vec2(nodePos[nodes.get(z)].x, nodePos[nodes.get(z)].y);
    
    }
    agentVel[j] = goalPosition[j].minus(startingPos[j]);
    if (agentVel[j].length() > 0)
      agentVel[j].setToLength(goalSpeed);
    path.get(j)[nodes.size()-1]=goalPosition[j];
    println(path.get(j));
    len = path.get(j).length;
    posPlace[j] = 0;
  }
  }
  if (keyCode == DOWN){
    path = new ArrayList<Vec2[]>();
        posPlace = new int[maxNumNodes];

    circlePos[0].y += speed;
      connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes);
  // path = new ArrayList(numNodes);
  for(int j = 0;j<numOfAgents;j++){
    nodes = planPath(startingPos[j], goalPosition[j], circlePos, circleRad, numObstacles, nodePos, numNodes);
    println(nodes);
    Vec2 [] temp =  new Vec2[nodes.size()];
    path.add(temp);
    for(int z  = 0;z<nodes.size();z++){
      // println(nodePos[z-1].x, nodePos[z-1].y,nodePos[z].x, nodePos[z].y,nodePos[z+1].x, nodePos[z+1].y);
    println(path.get(j),nodePos[nodes.get(z)]);
    path.get(j)[z]=new Vec2(nodePos[nodes.get(z)].x, nodePos[nodes.get(z)].y);
    
    }
    agentVel[j] = goalPosition[j].minus(startingPos[j]);
    if (agentVel[j].length() > 0)
      agentVel[j].setToLength(goalSpeed);
    path.get(j)[nodes.size()-1]=goalPosition[j];
    println(path.get(j));
    len = path.get(j).length;
    posPlace[j] = 0;
  }
  }
}
//For real time user interaction

void mousePressed(){
 if (mouseButton == RIGHT){
   Vec2 temp = new Vec2(mouseX, mouseY);
  //   fill(20,200,150);

  //  circle(temp.x, temp.y, agentRad*2); 
   startingPos[numOfAgents] = temp;
   firstPos[numOfAgents] = temp;
    goalPos = sampleFreePos();
    goalPosition[numOfAgents] = goalPos;
   numOfAgents = numOfAgents+1;
   //println("New Start is",startPos.x, startPos.y);
  path  = new ArrayList(numNodes);
  for(int j = 0;j<numOfAgents;j++){
    nodes = planPath(startingPos[j], goalPosition[j], circlePos, circleRad, numObstacles, nodePos, numNodes);
    if(nodes.get(0)==-1){
      while(nodes.get(0)==-1){
      agentPos= sampleFreePos();
      startPos = new Vec2(agentPos.x, agentPos.y);
      startingPos[j]=startPos;
      firstPos[j] = startPos;
      // numOfAgents= numOfAgents+1;
      goalPos = sampleFreePos();
      goalPosition[j] = goalPos;
      nodes = planPath(startingPos[j], goalPosition[j], circlePos, circleRad, numObstacles, nodePos, numNodes);

      }

    }
    println(nodes);
    Vec2 [] temp1 =  new Vec2[nodes.size()+1];
    path.add(temp1);
    for(int z  = 0;z<nodes.size();z++){
      // println(nodePos[z-1].x, nodePos[z-1].y,nodePos[z].x, nodePos[z].y,nodePos[z+1].x, nodePos[z+1].y);
    path.get(j)[z]=new Vec2(nodePos[nodes.get(z)].x, nodePos[nodes.get(z)].y);
    
    }
    agentVel[j] = goalPosition[j].minus(startingPos[j]);
    if (agentVel[j].length() > 0)
      agentVel[j].setToLength(goalSpeed);
  
    path.get(j)[nodes.size()]=goalPosition[j];
    println(path.get(j));
    len = path.get(j).length;
    posPlace[j] = 0;
  }
 }
 else{
  //  goalPos = new Vec2(mouseX, mouseY);
   //println("New Goal is",goalPos.x, goalPos.y);
 }
//  curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
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
//When will agents 1 and 2 collide if they keep their current velocities?
float computeTTC(Vec2 pos1, Vec2 vel1, float radius1, Vec2 pos2, Vec2 vel2, float radius2){
  //return -1;
  float combinedRadius = radius1+radius2;
  Vec2 relativeVelocity = vel1.minus(vel2);
  float ttc = rayCircleIntersectTime(pos2, combinedRadius, pos1, relativeVelocity);
  if (ttc==0) println(ttc);
  return ttc;
}

// Compute attractive forces to draw agents to their goals, 
// and avoidance forces to anticipatory avoid collisions
Vec2 computeAgentForces(int id){
  //TODO: Make this better
  Vec2 acc = new Vec2(0,0);
  
  
  Vec2 goalVel = path.get(id)[posPlace[id]].minus(startingPos[id]);
  if (goalVel.length() > goalSpeed) goalVel.setToLength(goalSpeed);
  Vec2 goalForce = (goalVel.minus(agentVel[id]));
  acc.add(goalForce.times(k_goal));
  
  if (goalVel.length() < 5) return acc;
  

  for (int j = 0; j < numOfAgents; j++){
    if (j == id) continue;
    float ttc = computeTTC(startingPos[id],agentVel[id], agentRad, startingPos[j],agentVel[j], agentRad);
    
    Vec2 futurePos_id = startingPos[id].plus(agentVel[id].times((ttc+0.00001)));
    Vec2 futurePos_j = startingPos[j].plus(agentVel[j].times((ttc+0.00001)));
    Vec2 avoidDir = futurePos_id.minus(futurePos_j).normalized();
    Vec2 avoidForce = avoidDir.times(1/(ttc+0.0001));
    //println(ttc, avoidForce.x, avoidForce.y);
    if (ttc > 0){
      acc.add(avoidForce.times(k_avoid));
      if(acc.x<0){
        acc.x = max(-100,acc.x);
      }
      if(acc.x>0){
        acc.x = min(100,acc.x);
      }
      if(acc.y<0){
        acc.y = max(-100,acc.y);
      }
      if(acc.y>0){
        acc.y = min(100,acc.y);
      }
    }
    if (ttc==0){
      acc.add(avoidForce.times(k_avoid));
      if(acc.x<0){
        acc.x = max(-100,acc.x);
      }
      if(acc.x>0){
        acc.x = min(100,acc.x);
      }
      if(acc.y<0){
        acc.y = max(-100,acc.y);
      }
      if(acc.y>0){
        acc.y = min(100,acc.y);
      }
    }
  }
  for (int j = 0; j < numCollisions; j++){
    float ttc = computeTTC(startingPos[id],agentVel[id], agentRad, circlePos[j],new Vec2(0,0), circleRad[j]);
    
    Vec2 futurePos_id = startingPos[id].plus(agentVel[id].times((ttc+0.00001)));
    Vec2 futurePos_j = circlePos[j].plus(new Vec2(0,0));
    Vec2 avoidDir = futurePos_id.minus(futurePos_j).normalized();
    Vec2 avoidForce = avoidDir.times(1/(ttc+0.0001));
    //println(ttc, avoidForce.x, avoidForce.y);
    if (ttc > 0){
      acc.add(avoidForce.times(k_avoid));
      if(acc.x<0){
        acc.x = max(-100,acc.x);
      }
      if(acc.x>0){
        acc.x = min(100,acc.x);
      }
      if(acc.y<0){
        acc.y = max(-100,acc.y);
      }
      if(acc.y>0){
        acc.y = min(100,acc.y);
      }
    }
    if (ttc==0){
      acc.add(avoidForce.times(k_avoid));
      if(acc.x<0){
        acc.x = max(-100,acc.x);
      }
      if(acc.x>0){
        acc.x = min(100,acc.x);
      }
      if(acc.y<0){
        acc.y = max(-100,acc.y);
      }
      if(acc.y>0){
        acc.y = min(100,acc.y);
      }
    }
  }
  
  return acc;
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
