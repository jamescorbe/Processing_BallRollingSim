///////////////////////////////////////////////////////////////
// This example builds on SimSphereMover1_CodeExample.
// The changes are only in the first tab. The SimSphereMover and other tabs remains unaltered.
// It adds in how you would detect collision between 2 SimSphereMover objects,
// and how you would use the physics component of the SimSphereMover to produce
// the collision response - i.e. how they bounce off each other.

// The scene is set up so that if you just press the "W" key, ball1 sould move and collide with ball2
SimObjectManager simObjectManager = new SimObjectManager();
SimSurfaceMesh terrain; 
SimSphereMover ball1;
SimSphereMover ball2;

SimpleUI UI;
SimSphereMover moveBall;
PImage image;
int terrainWidth;
float terrainHeight =0;
PVector ballMotionVector; 
PVector force;
String terrainImg = "hill2.jpg";
PVector Gravity;
PVector objectVelocity;
PVector minVelociy;
int numberOfBalls;
PVector minVel;

SimCamera myCamera;

void setup(){
  UI = new SimpleUI();
  numberOfBalls = 5;
  minVelociy = new PVector(0.1,0.1,0.1);
  size(900, 700, P3D);
  
  String[] menuItems = {"save","load","quit"};
  UI.addMenu("File", 20,20, menuItems);
  UI.addToggleButton("move", 20,45);
  UI.addToggleButton("reset", 20,70);
  UI.addSlider("height", 20,240).setSliderValue(0);
  UI.addSlider("width", 20,100).setSliderValue(0);
  
  ball1 = new SimSphereMover(vec(0,-100,40), 8.0f);
  ball2 = new SimSphereMover(vec(0,-100,0), 8.0f);
  
  
  ////////////////////////////
  // create the SimCamera
  myCamera = new SimCamera();
  myCamera.setPositionAndLookat(vec(  -66.66498, -37.02684, 175.28958),vec(-66.19523, -37.103973, 174.41016)); 
  myCamera.setHUDArea(20,20,200,200);
  myCamera.isMoving = false;
  
  
  ////////////////////////////
  // add a terrian, made from 20x20 facets, each 20 units in size
  // shifted so that it is centered around the scene's origin
  terrain = new SimSurfaceMesh(20, 20, 20.0);
  image = loadImage("hill2.jpg");
  terrain.setHeightsFromImage(image, 200);
  terrain.setTransformAbs( 1, 0,0,0, vec(-205,0,-205));
  minVel = new PVector(0,0,0);
  ballMotionVector = new PVector(0,1,0.1);
  Gravity = new PVector(0,40,0);
  
  
 // simObjectManager.addSimObject(terrain,"table");
  makeSomeBalls();
  
  
}



void draw(){
  background(0);
  lights();
  
  // draw the 3d stuff
  stroke(255,255,255);
  fill(150,200, 150 );
  noStroke();
  terrain.drawMe(); 

  fill(255,0,0);
  //ball1.drawMe();
  //ball2.drawMe();
  ball1.physics.addForce(Gravity);

  ballCollision();
  updateBallDrop();
  ball1.physics.applyFriction();
  myCamera.update();
  drawMajorAxis(new PVector(0,0,0), 200); 
  drawTheballs();
  
   myCamera.startDrawHUD();
    // any 2d drawing has to happen between 
    // startDrawHUD and endDrawHUD
    UI.update();
  myCamera.endDrawHUD();
}

void handleUIEvent(UIEventData uied){
  // here we just get the event to print its self
  // with "verbosity" set to 1, (2 = medium, 3 = high, 0 = do not print anything)
  uied.print(2);
  
  
  if(uied.eventIsFromWidget("reset") ){
    simObjectManager.clearBalls();
    makeSomeBalls();
  }
  
  if(uied.eventIsFromWidget("height") ){
    float amount = uied.sliderValue * 400;
    terrainHeight = int(amount);
    terrain.setHeightsFromImage(image, amount);
  }
  if(uied.eventIsFromWidget("width") ){
    SimSurfaceMesh terrain2;
    float amount = uied.sliderValue *400;
    terrainWidth = int(amount);
    terrain2 = new SimSurfaceMesh(20, terrainWidth, 20.0);
    if(terrainHeight ==0){
    
    terrain2.setHeightsFromImage(image, 200);
  }
  else{
     terrain2.setHeightsFromImage(image, terrainHeight);
  }
   
    terrain = terrain2;
    terrain.setTransformAbs( 1, 0,0,0, vec(-205,0,-205));
  }
  
  /////////////////////////////////////////////////////////////
  // this stuff opens the file dialogue window, and then when you have
  // clicked the "open" button, ....
  if(uied.menuItem.equals("load") ){
    UI.openFileLoadDialog("load a texture");
  }
  
  // .... this bit happens
  if(uied.eventIsFromWidget("fileLoadDialog") ){
    String fileName = uied.fileSelection;
    terrain.textureMap = loadImage(fileName);
  }
  /////////////////////////////////////////////////////////////
  
}


void makeSomeBalls(){
 
  for(int n = 0; n < numberOfBalls; n++){
    float radius = random(4,9);
    float x = random(0,10);
    float y = random(0,20);
    float z = random(0,40);
    SimSphereMover planet = new SimSphereMover(vec(x,y,z), radius);
     planet.levelOfDetail = 8;
   
    simObjectManager.addSphereObject(planet, "ball_" + n);
    print(simObjectManager.getSimSphereObjects());
   
  }
  }
  
  void drawTheballs(){
    for(int n = 0; n < numberOfBalls; n++){
    noStroke();
    fill(random(255),random(255),random(255));
    simObjectManager.getSimSphereObject(n).drawMe();
    objectVelocity = simObjectManager.getSimSphereObject(n).physics.velocity;
    simObjectManager.getSimSphereObject(n).physics.simulateGravity();
    
  }
 
  noStroke();
  fill(127,127,127);
  
  randomSeed(1);
  
}
  
  
void ballCollision(){
  
  for (int i=0; i < numberOfBalls; i++)
 {
   
    for(int x=0; x < numberOfBalls; x++)
    {
      objectVelocity = simObjectManager.getSimSphereObject(x).physics.velocity;
      if( simObjectManager.getSimSphereObject(i).collidesWith(simObjectManager.getSimSphereObject(x)) ){
    println("colliding");
    simObjectManager.getSimSphereObject(i).physics.collisionResponse(simObjectManager.getSimSphereObject(x).physics);
     if((objectVelocity.x > minVelociy.x && objectVelocity.y > minVelociy.y && objectVelocity.z > minVelociy.z) && !simObjectManager.getSimSphereObject(i).physics.getisGravity())
      {
        simObjectManager.getSimSphereObject(x).physics.toggleisGravity();
      }
    drawTheballs();

      }
      
    }
 }
}

  

void updateBallDrop(){
 for (int i=0; i < numberOfBalls; i++)
 {
   //print("update");
    simObjectManager.getSimSphereObject(i);
   
  ballMotionVector = simObjectManager.getSimSphereObject(i).physics.velocity;
 float speed = ballMotionVector.mag();
  
  // cast a ray out from the ball along its line of travel
  SimRay ballray = new SimRay( simObjectManager.getSimSphereObject(i).getOrigin(), PVector.add(simObjectManager.getSimSphereObject(i).getOrigin(), ballMotionVector));
  drawray(ballray);
  
  // see if it intersects the terrain
  if( ballray.calcIntersection(terrain))
  {

    PVector intersectionPt = ballray.getIntersectionPoint();
    PVector intersectionNormal = ballray.getIntersectionNormal();

    PVector reflectionVector = reflectOffSurface(ballMotionVector, intersectionNormal);
    SimRay reflectionRay = new SimRay(intersectionPt, PVector.add(intersectionPt, reflectionVector));
    drawray(reflectionRay);
  
    // now see if it is colliding with the terrain yet
    if( simObjectManager.getSimSphereObject(i).isPointInside(  intersectionPt ) ) 
    {
      // calculate new motion vector
      //ballMotionVector = reflectionVector.copy();
      PVector unitReflect = reflectionRay.direction.normalize();
      PVector velocityOnBounce = PVector.mult(unitReflect, speed);
      //PVector  RVincreased = PVector.mult(reflectionVector.copy(), 3000);
      // PVector resultForce = PVector.add(inverse, RVincreased);
      //ball1.physics.addForce(resultForce);
      
      simObjectManager.getSimSphereObject(i).physics.velocity = velocityOnBounce;
      if (velocityOnBounce.x < minVelociy.x && velocityOnBounce.y < minVelociy.y && velocityOnBounce.z < minVelociy.z)
      {
        //simObjectManager.getSimSphereObject(i).physics.toggleisGravity();
        simObjectManager.getSimSphereObject(i).physics.velocity = new PVector(0,0,0);
        //simObjectManager.getSimSphereObject(i).physics.addForce(new PVector(0,-41,0));
      }
      else if ((velocityOnBounce.x > minVelociy.x && velocityOnBounce.y > minVelociy.y && velocityOnBounce.z > minVelociy.z) && !simObjectManager.getSimSphereObject(i).physics.getisGravity())
      {
        simObjectManager.getSimSphereObject(i).physics.toggleisGravity();
      }
      else
      {
        
      }
      
    }
      
  }
  objectVelocity =simObjectManager.getSimSphereObject(i).physics.velocity;
  if(objectVelocity.x > minVelociy.x && objectVelocity.y > minVelociy.y && objectVelocity.z > minVelociy.z)
  {
    simObjectManager.getSimSphereObject(i).physics.simulateGravity();
  }
  else
  {
    
  }
    
 }
  
}


PVector reflectOffSurface(PVector incident, PVector surfaceNormal){
  // Using Fermat's Principle: the formula R = 2(N.I)N-I, where N is the surface normal,
  // I is the vector of the incident ray, and R is the resultant reflection.
  // First make sure you are working on copies only so changes are not propagated outside the method,
  // and that the incident and surfaceNormal vectors are normalized
  
  
  PVector i = incident.copy();
  i.normalize();
  
  PVector n = surfaceNormal.copy();
  n.normalize();
  
  // do the vector maths
  float n_dot_i_x2 = n.dot(i)*2;
  
  PVector n_dot_i_x2_xn = PVector.mult(n, n_dot_i_x2);
  
  PVector reflection =  PVector.sub(i, n_dot_i_x2_xn);
  // need to do this to create a reflection "this side" of the surface, not the ray on the other side
  //reflection.mult(-1);
  //println("reflect I,SN,R ", i, n, reflection);
  return reflection;
 
  
}







void drawray(SimRay r){
  PVector farPoint = r.getPointAtDistance(1000);
  pushStyle();  
  stroke(255,100,100); 
  line(r.origin.x, r.origin.y, r.origin.z, farPoint.x, farPoint.y, farPoint.z);
  popStyle();
}





void keyPressed(){

  float force = 1000f;
  
  if(key == 'c'){ 
     // toggle the camera isActive field
     myCamera.isMoving = !myCamera.isMoving;
  }

  // if the camera is active don't want the oject to move
  // so return now
  if( myCamera.isMoving ){
    //println("camera pos ", myCamera.cameraPos, " looKat ", myCamera.cameraLookat);
    return;
  }
  
  if(key == 'w'){
    // away from the user
    moveObject(0,0, -force);
    }
  
  if(key == 's'){
    // towards the user
    moveObject(0,0,force);
    }
  
  if(key == CODED){
    if(keyCode == LEFT){
      moveObject(-force,0,0);
      }
    if(keyCode == RIGHT){ 
     moveObject(force,0,0);
      }
    if(keyCode == UP){
      moveObject(0,-force, 0);
      }
     if(keyCode == DOWN){
      moveObject(0,force, 0);
       }  
    }
}

void moveObject(float x, float y, float z){
 
  PVector force = new PVector(x, y, z);
  
  ball1.physics.addForce(force);
 
}
