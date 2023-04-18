///////////////////////////////////////////////////////////////
// This example builds on SimSphereMover1_CodeExample.
// The changes are only in the first tab. The SimSphereMover and other tabs remains unaltered.
// It adds in how you would detect collision between 2 SimSphereMover objects,
// and how you would use the physics component of the SimSphereMover to produce
// the collision response - i.e. how they bounce off each other.

// The scene is set up so that if you just press the "W" key, ball1 sould move and collide with ball2
SimObjectManager simObjectManager = new SimObjectManager();
SimSurfaceMesh terrain; 
SimSurfaceMesh floor; 
SimSphereMover ball1;
SimSphereMover ball2;
SimModel BackGround;
SimpleUI UI;
SimSphereMover moveBall;
PImage image;
int terrainWidth;
int newBallCount;
boolean simRayDraw = false;
float terrainHeight =0;
PVector ballMotionVector; 
PVector force;
String terrainImg = "hill2.jpg";
float newGravity;
String[] hills = {"round Hill","crater","spikes","slide"};
int ballSpaceX;
int minSize = 4;
int maxSize = 15;
int ballSpaceY;
int ballSpaceZ;
PVector objectVelocity;
PVector minVelociy;
int numberOfBalls;
PVector minVel;

SimCamera myCamera;

void setup(){
  frameRate(400);
  UI = new SimpleUI();
  numberOfBalls = 20;
  minVelociy = new PVector(0.091,0.091,0.091);
  size(displayWidth, displayHeight, P3D);
  BackGround = new SimModel("G4G_BG.obj");
  UI.addSimpleButton("reset", 20,20);
  UI.addSlider("height", 20,55).setSliderValue(0);
  UI.addSlider("width", 20,90).setSliderValue(0);
  UI.addTextInputBox("Balls", 20,130).setText("20");
  UI.addTextInputBox("min Size", 20,165).setText("4");
  UI.addTextInputBox("max Size", 20,200).setText("15");
  UI.addSimpleButton("Update Balls",20,235);
  UI.addTextInputBox("Gravity", 20,270).setText("9.8");
  UI.addSimpleButton("Update Gravity",20,305);
  UI.addMenu("Change Hill",20,340,hills);
  BackGround.setTransformAbs(10, 3.1415,0,0, vec(-305,210,-205));
  BackGround.showBoundingVolume(false);
  ball1 = new SimSphereMover(vec(0,-100,40), 8.0f);
  ball2 = new SimSphereMover(vec(0,-100,0), 8.0f);
  
  
  ////////////////////////////
  // create the SimCamera
  myCamera = new SimCamera();
  myCamera.setPositionAndLookat(vec( -470.6324, -179.39978, -20.768616),vec(-65.19523, -35.103973, 10.41016)); 
  myCamera.setHUDArea(20,20,200,320);
  myCamera.isMoving = false;
  
  floor = new SimSurfaceMesh(30, 30, 30.0);
  ////////////////////////////
  // add a terrian, made from 20x20 facets, each 20 units in size
  // shifted so that it is centered around the scene's origin
  terrain = new SimSurfaceMesh(30, 30, 30.0);
  image = loadImage("circle2.jpg");
  terrain.setHeightsFromImage(image, 200);
  terrain.setTransformAbs( 1, 0,0,0, vec(-405,-100,-255));
  floor.setTransformAbs( 1, 0,0,0, vec(-205,115,-205));
  minVel = new PVector(0,0,0);
   simObjectManager.addSimObject(floor, "floor");
  ballMotionVector = new PVector(0,1,0.1);

  
 // simObjectManager.addSimObject(terrain,"table");
  makeSomeBalls();
  
}
void updateBallGravity()
{
  for(int n = 0; n < numberOfBalls; n++){
    print("Grav  update");
    print(simObjectManager.getSimSphereObject(n).physics.inGravity);
    simObjectManager.getSimSphereObject(n).physics.inGravity = newGravity;
    simObjectManager.getSimSphereObject(n).physics.updateGravity();
    
  }
}

void draw(){
  background(0);
  lights();
  
  // draw the 3d stuff
  stroke(255,255,255);
  fill(150,200, 150 );
  noStroke();
  terrain.drawMe(); 
  floor.drawMe();
  BackGround.drawMe();
  fill(255,0,0);
  //ball1.drawMe();
  //ball2.drawMe();

  ballCollision();
  
 //drawMajorAxis(new PVector(0,0,0), 200); 
  drawTheballs();
  updateBallDrop();
  myCamera.update();
   myCamera.startDrawHUD();
    // any 2d drawing has to happen between 
    // startDrawHUD and endDrawHUD
    myCamera.setHUDArea(20,20,200,330);
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

 
  if(uied.eventIsFromWidget("Update Balls"))
  {
    try{
    newBallCount = int(UI.getText("Balls"));
    minSize = int(UI.getText("min Size"));
    maxSize = int(UI.getText("max Size"));
    if(newBallCount <200)
      {
        numberOfBalls = newBallCount;
        simObjectManager.clearBalls();
        makeSomeBalls();
        
      }
    else
      {
        print("ball count too high 200 is the maximum!");
      }
   
    }
    catch(Exception e)
    {
      print(e);
    }
  }
  if(uied.eventIsFromWidget("Update Gravity"))
  {
    try
    {
      float gravity = float(UI.getText("Gravity"));
      gravity = (gravity * 8.1632653); 
 
      newGravity = gravity;
      updateBallGravity();
    }
    catch(Exception e)
    {
      
    }
  
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
 randomSeed(millis());
  for(int n = 0; n < numberOfBalls; n++){
    float radius = random(minSize,maxSize);
    float x = random(-180,190);
    float y = random(-200,-40);
    float z = random(-160,-50);
    SimSphereMover planet = new SimSphereMover(vec(x,y,z), radius);
     planet.levelOfDetail = 8;
   
    simObjectManager.addSphereObject(planet, "ball_" + n);
    simObjectManager.getSimSphereObject(n).physics.updateMass(radius/2.5);
    print(simObjectManager.getSimSphereObjects());
    if(newGravity != 0)
    {
      simObjectManager.getSimSphereObject(n).physics.inGravity = newGravity;
      simObjectManager.getSimSphereObject(n).physics.updateGravity();
    }
   
  }
  }
  
  void drawTheballs(){
    for(int n = 0; n < numberOfBalls; n++){
    noStroke();
    fill(random(255),random(255),random(255));
    simObjectManager.getSimSphereObject(n).drawMe();
    objectVelocity = simObjectManager.getSimSphereObject(n).physics.velocity;
    float velocityMulti = objectVelocity.x + objectVelocity.y + objectVelocity.z;
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
    //println("colliding");
    simObjectManager.getSimSphereObject(i).physics.collisionResponse(simObjectManager.getSimSphereObject(x).physics);
    }
 }
 drawTheballs();
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
  if(simRayDraw == true)
    {
    //drawray(reflectionRay);
    }drawray(ballray);
  
  // see if it intersects the terrain
  if( ballray.calcIntersection(terrain) || ballray.calcIntersection(floor))
  {

    PVector intersectionPt = ballray.getIntersectionPoint();
    PVector intersectionNormal = ballray.getIntersectionNormal();

    PVector reflectionVector = reflectOffSurface(ballMotionVector, intersectionNormal);
    SimRay reflectionRay = new SimRay(intersectionPt, PVector.add(intersectionPt, reflectionVector));
    
    // now see if it is colliding with the terrain yet
    if( simObjectManager.getSimSphereObject(i).isPointInside(  intersectionPt ) ) 
    {
      simObjectManager.getSimSphereObject(i).physics.setterrainCollide(true);
      // calculate new motion vector
      //ballMotionVector = reflectionVector.copy();
      PVector unitReflect = reflectionRay.direction.normalize();
      PVector velocityOnBounce = PVector.mult(unitReflect, speed);
      velocityOnBounce = PVector.div(velocityOnBounce, (simObjectManager.getSimSphereObject(i).physics.mass)/2);
      //PVector  RVincreased = PVector.mult(reflectionVector.copy(), 3000);
      // PVector resultForce = PVector.add(inverse, RVincreased);
      //ball1.physics.addForce(resultForce);
      objectVelocity = simObjectManager.getSimSphereObject(i).physics.velocity;
      simObjectManager.getSimSphereObject(i).physics.velocity = velocityOnBounce;
      if (simObjectManager.getSimSphereObject(i).physics.isterrainCollide() )
      {
        float velocityMulti = objectVelocity.x + objectVelocity.y + objectVelocity.z;
        print(" ",velocityMulti ," ");
       
        
        if(velocityMulti >2 && simObjectManager.getSimSphereObject(i).physics.isGravity == true)
        {
         float  weightMulti = (simObjectManager.getSimSphereObject(i).physics.mass)*3;
         if(objectVelocity.x > minVelociy.x && objectVelocity.y > minVelociy.y)
            simObjectManager.getSimSphereObject(i).physics.addForce(new PVector(0,-(simObjectManager.getSimSphereObject(i).physics.inGravity*(weightMulti)*3),0));
         
         else{
          simObjectManager.getSimSphereObject(i).physics.addForce(new PVector(0,-(simObjectManager.getSimSphereObject(i).physics.inGravity*(weightMulti*0.9)),0));
          }
        }
      }
    }
      
  }
  else
  {
    simObjectManager.getSimSphereObject(i).physics.setterrainCollide(false);
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
    
    if(key == 'b'){
    // towards the user
    print(myCamera.getPosition());
    }
    if(key == 'r'){
    // towards the user
    simRayDraw = !simRayDraw;
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
