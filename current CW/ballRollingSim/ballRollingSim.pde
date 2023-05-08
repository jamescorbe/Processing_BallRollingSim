SimObjectManager simObjectManager = new SimObjectManager();
SimSurfaceMesh terrain; 
SimSurfaceMesh floor; 
SimSphereMover ball1;
SimSphereMover ball2;
SimModel BackGround;
SimpleUI UI;
SimSphereMover moveBall;
PImage image;
PImage grass;
PImage water;
PImage sky;
int terrainWidth;
int newBallCount;
boolean simRayDraw = false;
float terrainHeight =0;
PVector ballMotionVector; 
PVector force;
String terrainImg = "hill2.jpg";
float newGravity;
String[] hills = {"round Hill","long Hill","spikes","slide"};
int ballSpaceX;
int minSize = 4;
int maxSize = 15;
int ballSpaceY;
int ballSpaceZ;
String hillType;
PVector terrainLocation;
PVector objectVelocity;
PVector minVelociy;
float ballMinX;
float ballMaxX;
float ballMinY;
float ballMaxY;
float ballMinZ;
float ballMaxZ;
int numberOfBalls;
PVector minVel;

SimCamera myCamera;

void setup(){
  frameRate(400);
  UI = new SimpleUI();
  water = loadImage("water.jpg");
  grass = loadImage("grass.jpg");
  numberOfBalls = 20;
  directionalLight(128, 128, 128, 0, 0, -1);
  minVelociy = new PVector(0.03,0.03,0.03);
  size(displayWidth, displayHeight, P3D);
  BackGround = new SimModel("G4G_BG2.obj");
  UI.addSimpleButton("reset", 20,20);
  UI.addSlider("height", 20,55).setSliderValue(0);
  UI.addSlider("width", 20,90).setSliderValue(0);
  UI.addTextInputBox("Balls", 20,130).setText("20");
  UI.addTextInputBox("min Size", 20,165).setText("4");
  UI.addTextInputBox("max Size", 20,200).setText("15");
  UI.addSimpleButton("Update Balls",20,235);
  UI.addTextInputBox("Gravity", 20,270).setText("9.8");
  UI.addSimpleButton("Update Gravity",20,305);
  UI.addRadioButton("flat Hill",20,340,"hillType");
  UI.addRadioButton("round Hill",20,365,"hillType");
  BackGround.setTransformAbs(13, 3.1415,0,0, vec(-305,130,-205));
  BackGround.showBoundingVolume(false);
  ball1 = new SimSphereMover(vec(0,-100,40), 8.0f);
  ball2 = new SimSphereMover(vec(0,-100,0), 8.0f);
  terrainLocation = new PVector(-705,-100,-355);
  hillType ="hill2.jpg";
  ballMinX = -450;
  ballMaxX = -500;
  ballMinY = -240;
  ballMaxY = -250;
  ballMinZ = -160;
  ballMaxZ = -50;
  
  // create the SimCamera
  myCamera = new SimCamera();
  myCamera.setPositionAndLookat(vec( -1034.3253, -405.76044, -30.248709),vec(-65.19523, -35.103973, 10.41016)); 
  myCamera.setHUDArea(20,20,200,350);
  myCamera.isMoving = false;
  
  floor = new SimSurfaceMesh(40, 40, 40.0);

  terrain = new SimSurfaceMesh(25, 25, 25.0);
  image = loadImage(hillType);
  terrain.setHeightsFromImage(image, 200);
  terrain.setTransformAbs( 1, 0,0,0, terrainLocation);
  floor.setTransformAbs( 1, 0,0,0, vec(-905,107,-805));
  minVel = new PVector(0,0,0);
   simObjectManager.addSimObject(floor, "floor");
  ballMotionVector = new PVector(0,1,0.1);
  terrain.setTextureMap(grass);
  floor.setTextureMap(water);

  

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
  background(176, 241, 255);
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
  updateBallDrop();
  drawTheballs();
  
  myCamera.update();
   myCamera.startDrawHUD();
    
    myCamera.setHUDArea(20,20,200,385);
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
    terrain.setTextureMap(grass);
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
    terrain.setTransformAbs( 1, 0,0,0, terrainLocation);
    terrain.setTextureMap(grass);
  }
  if(uied.eventIsFromWidget("round Hill"))
  {
    image = loadImage("circle2.jpg");
    terrain.setHeightsFromImage(image, 200);
    terrain.setTransformAbs( 1, 0,0,0, terrainLocation);
    ballMinX = -450;
    ballMaxX = -500;
    ballMinY = -240;
    ballMaxY = -290;
    ballMinZ = -160;
    ballMaxZ = -50;
    simObjectManager.clearBalls();
    makeSomeBalls();
  }
  if(uied.eventIsFromWidget("flat Hill"))
  {
    image = loadImage("hill2.jpg");
    terrain.setHeightsFromImage(image, 200);
    terrain.setTransformAbs( 1, 0,0,0, terrainLocation);
    ballMinX = -600;
    ballMaxX =  -90;
    ballMinY = -280;
    ballMaxY = -150;
    ballMinZ = -250;
    ballMaxZ =  -150;
    simObjectManager.clearBalls();
    makeSomeBalls();
  
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
  // Creates the randomly balls in a location depending on map choice
 randomSeed(millis());
  for(int n = 0; n < numberOfBalls; n++){
    float radius = random(minSize,maxSize);
    float x = random(ballMinX, ballMaxX);
    float y = random(ballMinY, ballMaxY);
    float z = random(ballMinZ, ballMaxZ);
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
  
  //draws all balls in the object manager
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
  
  // handles collision for each ball in the object manager with the use of a nested loop.
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

  

void updateBallDrop(){// detects collisions with surfaces, to simulate a bounce while allowing rolling with the due to a force opposite gravity being applied for continuous collision.
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
    }//drawray(ballray);
  
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
      //if (simObjectManager.getSimSphereObject(i).physics.isterrainCollide() )
       if(simObjectManager.getSimSphereObject(i).isPointInside(  intersectionPt ))
      {
        float velocityMulti = objectVelocity.x + objectVelocity.y + objectVelocity.z;
        print(" ",objectVelocity.z ," ");
       
        
          if(velocityMulti >2 && simObjectManager.getSimSphereObject(i).physics.isGravity == true)
        {
         float antiGravity = ((simObjectManager.getSimSphereObject(i).physics.inGravity * -1))*simObjectManager.getSimSphereObject(i).physics.mass;
         
         if(objectVelocity.z >0.1 || objectVelocity.z <0)
                {
                simObjectManager.getSimSphereObject(i).physics.addForce(new PVector(0,-(simObjectManager.getSimSphereObject(i).physics.inGravity*25),0));
                }
         
          else{
               simObjectManager.getSimSphereObject(i).physics.addForce(new PVector(0,(antiGravity)*2,0));
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
    print("cam pos",myCamera.getPosition());
    }
    if(key == 'r'){
    // towards the user
    simRayDraw = !simRayDraw;
    }
  
}

void moveObject(float x, float y, float z){
 
  PVector force = new PVector(x, y, z);
  
  ball1.physics.addForce(force);
 
}
