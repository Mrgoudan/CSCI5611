//You will only be turning in this file
//Your solution will be graded based on it's runtime (smaller is better), 
//the optimality of the path you return (shorter is better), and the
//number of collisions along the path (it should be 0 in all cases).

//You must provide a function with the following prototype:
// ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes);
// Where: 
//    -startPos and goalPos are 2D start and goal positions
//    -centers and radii are arrays specifying the center and radius of obstacles
//    -numObstacles specifies the number of obstacles
//    -nodePos is an array specifying the 2D position of roadmap nodes
//    -numNodes specifies the number of nodes in the PRM
// The function should return an ArrayList of node IDs (indexes into the nodePos array).
// This should provide a collision-free chain of direct paths from the start position
// to the position of each node, and finally to the goal position.
// If there is no collision-free path between the start and goal, return an ArrayList with
// the 0'th element of "-1".

// Your code can safely make the following assumptions:
//   - The function connectNeighbors() will always be called before planPath()
//   - The variable maxNumNodes has been defined as a large static int, and it will
//     always be bigger than the numNodes variable passed into planPath()
//   - None of the positions in the nodePos array will ever be inside an obstacle
//   - The start and the goal position will never be inside an obstacle

// There are many useful functions in CollisionLibrary.pde and Vec2.pde
// which you can draw on in your implementation. Please add any additional 
// functionality you need to this file (PRM.pde) for compatabilty reasons.

// Here we provide a simple PRM implementation to get you started.
// Be warned, this version has several important limitations.
// For example, it uses BFS which will not provide the shortest path.
// Also, it (wrongly) assumes the nodes closest to the start and goal
// are the best nodes to start/end on your path on. Be sure to fix 
// these and other issues as you work on this assignment. This file is
// intended to illustrate the basic set-up for the assignmtent, don't assume 
// this example funcationality is correct and end up copying it's mistakes!).



//Here, we represent our graph structure as a neighbor list
//You can use any graph representation you like
ArrayList<Integer>[] neighbors = new ArrayList[maxNumNodes];  //A list of neighbors can can be reached from a given node
//We also want some help arrays to keep track of some information about nodes we've visited
Boolean[] visited = new Boolean[maxNumNodes]; //A list which store if a given node has been visited
int[] parent = new int[maxNumNodes]; //A list which stores the best previous node on the optimal path to reach this node

//Set which nodes are connected to which neighbors (graph edges) based on PRM rules
void connectNeighbors(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween);
      if (!circleListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }
}

// int connectStartGoalNeighbors(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes,Vec2 start,Vec2 goal){
  // int sID = numNodes;
  // nodePos[numNodes] = start;
  // neighbors[numNodes] = new ArrayList<Integer>();  //Clear neighbors list

  // numNodes = numNodes+1;
  // nodePos[numNodes] = goal;
  // neighbors[numNodes] = new ArrayList<Integer>();  //Clear neighbors list

  // numNodes = numNodes+1;


  // for (int i = numNodes-2; i < numNodes; i++){
  //   // neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
  //   for (int j = 0; j < numNodes; j++){
  //     if (i == j) continue; //don't connect to myself 
  //     Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
  //     // Vec2 dir1 = nodePos[i].minus(nodePos[j]).normalized();
  //     float distBetween = nodePos[i].distanceTo(nodePos[j]);
  //     hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween);
  //     // hitInfo circleListCheck1 = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween);

  //     if (!circleListCheck.hit&& distBetween<200){
  //       neighbors[i].add(j);
  //       neighbors[j].add(i);
  //       println(neighbors[j],neighbors[i]);
  //     }
  //   }
  // }

//   return sID;
// }


//This is probably a bad idea and you shouldn't use it...
int closestNode(Vec2 point, Vec2[] nodePos, int numNodes){
  int closestID = -1;
  float minDist = 999999;
  for (int i = 0; i < numNodes; i++){
    float dist = nodePos[i].distanceTo(point);
    if (dist < minDist){
      closestID = i;
      minDist = dist;
    }
  }
  return closestID;
}

int closestStartNode(Vec2 start,Vec2 goal ,Vec2[] nodePos, int numNodes,Vec2[] centers, float[] radii, int numObstacles){
  // int closestID = -1;
  // float minDist = 999999;
  float minDist = 99999999;
  float minDIstToGoal = 99999999;
  int closestID = -1;
  for (int i = 0; i < numNodes; i++){
    float dist = nodePos[i].distanceTo(start);
    float distToGoal = nodePos[i].distanceTo(goal);
    
    Vec2 dir = nodePos[i].minus(start).normalized();
    float distBetween = nodePos[i].distanceTo(start);
    hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, start, dir, distBetween);
    if(!circleListCheck.hit &&dist+distToGoal<minDist+minDIstToGoal){
      println(dist+distToGoal,minDist+minDIstToGoal);
      closestID = i;
      minDist = dist;
      minDIstToGoal = distToGoal;
    }
  }
  return closestID;
}
int closestGoalNode(Vec2 start,Vec2 goal ,Vec2[] nodePos, int numNodes,Vec2[] centers, float[] radii, int numObstacles){
  // int closestID = -1;
  // float minDist = 999999;
  float minDist = 99999999;
  float minDIstToGoal = 99999999;
  int closestID = -1;
  for (int i = 0; i < numNodes; i++){
    float dist = nodePos[i].distanceTo(goal);
    float distToGoal = nodePos[i].distanceTo(start);
    Vec2 dir = nodePos[i].minus(goal).normalized();
    float distBetween = nodePos[i].distanceTo(goal);
    hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, goal, dir, distBetween);
    if(!circleListCheck.hit &&dist+distToGoal<minDist+minDIstToGoal){
      closestID = i;
      minDist = dist;
      minDIstToGoal = distToGoal;
    }
  }
  return closestID;
}

ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  ArrayList<Integer> path = new ArrayList();
  
  int startID = closestStartNode(startPos,goalPos, nodePos, numNodes,centers,radii,numObstacles);
  int goalID = closestGoalNode(startPos,goalPos, nodePos, numNodes,centers,radii,numObstacles);
  
  path = runAStar(nodePos, numNodes, startID, goalID);
  println("path",path);
  
  return path;
}

int peek(ArrayList<Integer> openList, float[] fValue){
  int minID = 0;
  float minFValue = 99999;
  
  for(int i  = 0;i<openList.size();i++){
    // println("openlist size",openList.get(i));
    // println("peek value",openList, fValue[openList.get(i)]);
    if(fValue[openList.get(i)]<minFValue){
      minFValue = fValue[openList.get(i)];
      minID =openList.get(i) ;
    }
  }
  
  // println(minID);
  return minID;
}
ArrayList<Integer> runAStar(Vec2[] nodePos, int numNodes, int startID, int goalID){
  ArrayList<Integer> closedList = new ArrayList();
  ArrayList<Integer> openList = new ArrayList();
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
  }
  float[] fValue = new float[numNodes]; 
  float[] gValue = new float[numNodes];
  fValue[startID] = 0+nodePos[startID].distanceTo(nodePos[goalID]);
  gValue[startID] = 0;
  visited[startID] = true;
  openList.add(startID);
  // println("startID",startID);
  while(!openList.isEmpty()){
    int evaluate = peek(openList,fValue);
    if(evaluate == goalID){
      println("Goal Found");
      break;
    }
    for(int i = 0;i<neighbors[evaluate].size();i++){
      visited[neighbors[evaluate].get(i)] = true;
      int node = neighbors[evaluate].get(i);
      float totalWeight = gValue[evaluate]+nodePos[evaluate].distanceTo(nodePos[node]);
      if(!openList.contains(node)&& !closedList.contains(node)){
        parent[node] = evaluate;
        gValue[node] = totalWeight;
        fValue[node] = gValue[node] + nodePos[evaluate].distanceTo(nodePos[goalID]);
        openList.add(node);
      }else{
          if(totalWeight<gValue[node]){
            parent[node] = evaluate;
            gValue[node] = totalWeight;
            fValue[node] = gValue[node] + nodePos[evaluate].distanceTo(nodePos[goalID]);
            if(closedList.contains(node)){
              closedList.remove(closedList.indexOf(node));
              openList.add(node);
            }
        }
      }  
    }
    // println(openList, evaluate);
    openList.remove(openList.indexOf(evaluate));
    closedList.add(evaluate);
   }
  ArrayList<Integer> path = new ArrayList();
  int goal = goalID;
  while(true){
    // println("goal",goal);
    if(parent[goal]!=-1){
      path.add(0,goal);
    }else{
      path.add(0,goal);
      break;
    }
    
    goal = parent[goal];

  }
  // println(startID,goalID);

  // println(path);

  return path;
  
}

//BFS (Breadth First Search)
// ArrayList<Integer> runBFS(Vec2[] nodePos, int numNodes, int startID, int goalID){
//   ArrayList<Integer> fringe = new ArrayList();  //New empty fringe
//   ArrayList<Integer> path = new ArrayList();
//   for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
//     visited[i] = false;
//     parent[i] = -1; //No parent yet
//   }

//   //println("\nBeginning Search");
  
//   visited[startID] = true;
//   fringe.add(startID);
//   //println("Adding node", startID, "(start) to the fringe.");
//   //println(" Current Fringe: ", fringe);
  
//   while (fringe.size() > 0){
//     int currentNode = fringe.get(0);
//     fringe.remove(0);
//     if (currentNode == goalID){
//       //println("Goal found!");
//       break;
//     }
//     for (int i = 0; i < neighbors[currentNode].size(); i++){
//       int neighborNode = neighbors[currentNode].get(i);
//       if (!visited[neighborNode]){
//         visited[neighborNode] = true;
//         parent[neighborNode] = currentNode;
//         fringe.add(neighborNode);
//         //println("Added node", neighborNode, "to the fringe.");
//         //println(" Current Fringe: ", fringe);
//       }
//     } 
//   }
  
//   if (fringe.size() == 0){
//     //println("No Path");
//     path.add(0,-1);
//     return path;
//   }
    
//   //print("\nReverse path: ");
//   int prevNode = parent[goalID];
//   path.add(0,goalID);
//   //print(goalID, " ");
//   while (prevNode >= 0){
//     //print(prevNode," ");
//     path.add(0,prevNode);
//     prevNode = parent[prevNode];
//   }
//   //print("\n");
  
//   return path;
// }




// ======================= Collision Library=======================//
boolean pointInCircle(Vec2 center, float r, Vec2 pointPos, float eps){
  float dist = pointPos.distanceTo(center);
  if (dist < r+eps){ //small safety factor
    return true;
  }
  return false;
}

//Returns true if the point is inside a list of circle
//You must consider a point as colliding if it's distance is <= eps
boolean pointInCircleList(Vec2[] centers, float[] radii, int numObstacles, Vec2 pointPos, float eps){
  
  for (int i = 0; i < numObstacles; i++){
    Vec2 center =  centers[i];
    float r = radii[i];
    if (pointInCircle(center,r,pointPos,eps)){
      return true;
    }
  }
  return false;
}


class hitInfo{
  public boolean hit = false;
  public float t = 9999999;
}

hitInfo rayCircleIntesect(Vec2 center, float r, Vec2 l_start, Vec2 l_dir, float max_t){
  hitInfo hit = new hitInfo();
    //Step 2: Compute W - a displacement vector pointing from the start of the line segment to the center of the circle
    Vec2 toCircle = center.minus(l_start);
    
    //Step 3: Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
    float a = 1;  //Length of l_dir (we normalized it)
    float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
    float c = toCircle.lengthSqr() - (r+strokeWidth)*(r+strokeWidth); //different of squared distances
    
    float d = b*b - 4*a*c; //discriminant 
    
    if (d >=0 ){ 
      //If d is positive we know the line is colliding, but we need to check if the collision line within the line segment
      //  ... this means t will be between 0 and the length of the line segment
      float t1 = (-b - sqrt(d))/(2*a); //Optimization: we only need the first collision
      float t2 = (-b + sqrt(d))/(2*a); //Optimization: we only need the first collision
      //println(hit.t,t1,t2);
      if (t1 > 0 && t1 < max_t){
        hit.hit = true;
        hit.t = t1;
      }
      else if (t1 < 0 && t2 > 0){
        hit.hit = true;
        hit.t = -1;
      }
      
    }
    
  return hit;
}

hitInfo rayCircleListIntersect(Vec2[] centers, float[] radii,  int numObstacles, Vec2 l_start, Vec2 l_dir, float max_t){
  hitInfo hit = new hitInfo();
  hit.t = max_t;
  for (int i = 0; i < numObstacles; i++){
    Vec2 center = centers[i];
    float r = radii[i];
    
    hitInfo circleHit = rayCircleIntesect(center, r, l_start, l_dir, max_t);
    if (circleHit.t > 0 && circleHit.t < hit.t){
      hit.hit = true;
      hit.t = circleHit.t;
    }
    else if (circleHit.hit && circleHit.t < 0){
      hit.hit = true;
      hit.t = -1;
    }
  }
  return hit;
}


//======================Vector Library=========================//
//CSCI 5611 Vector 2 Library [Example]
// Stephen J. Guy <sjguy@umn.edu>

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public float lengthSqr(){
    return x*x+y*y;
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}
