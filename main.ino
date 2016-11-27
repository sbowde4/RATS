
/*This code must be compiled in the Arduino IDE, available here. https://www.arduino.cc/en/Main/Software */

#include <NewPing.h>/*This library is neccessary for the distance sensor to function. It is on our github*/

#define TRIGGER_PIN  14
#define ECHO_PIN     15
#define MAX_DISTANCE 200

/*
Used to simplify input handling. Each letter corrsponds to a button control on the Arduino control app available on the app store.
*/
const char INPUT_FORWARDS = 'w';
const char INPUT_BACKWARDS = 's';
const char INPUT_LEFT = 'a';
const char INPUT_RIGHT = 'd';
const char INPUT_STOP = 'b';
const char INPUT_AI = 'c';
const char INPUT_LOWER_CRANE = 'f';
const char INPUT_RAISE_CRANE = 'g';


const int rightPol = 13;
const int leftPol = 12;
const int rightBrake = 8;
const int leftBrake = 9;
const int rightCurr = A0;
const int leftCurr = A1;
const int rightSpeed = 3;
const int leftSpeed = 11;



NewPing distancesensor(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);







/*
A Timer class to help with timing movements. Such as when in automatic movement, and it senses a wall
in front, it needs to know how long to go back for.. how long to turn... etc. etc.
*/
class Timer
{
public:
  //Initializes variables
  Timer();

  //The various clock actions
  void start();
  void stop();
  void pause();
  void unpause();

  //Gets the timer's time
  unsigned long getTicks();

  //Checks the status of the timer
  bool isStarted();
  bool isPaused();

private:
  //The clock time when the timer started
  unsigned long mStartTicks;

  //The ticks stored when the timer was paused
  unsigned long mPausedTicks;

  //The timer status
  bool mPaused;
  bool mStarted;
};


Timer::Timer()
{
  //Initialize the variables
  mStartTicks = 0;
  mPausedTicks = 0;

  mPaused = false;
  mStarted = false;
}

void Timer::start()
{
  
  mStarted = true;


  mPaused = false;


  mStartTicks = millis();
  mPausedTicks = 0;
}

void Timer::stop()
{

  mStarted = false;


  mPaused = false;


  mStartTicks = 0;
  mPausedTicks = 0;
}

void Timer::pause()
{

  if (mStarted && !mPaused)
  {
    //Pause the timer
    mPaused = true;

    mPausedTicks = millis() - mStartTicks;
    mStartTicks = 0;
  }
}

void Timer::unpause()
{

  if (mStarted && mPaused)
  {
  
    mPaused = false;

  
    mStartTicks = millis() - mPausedTicks;

    //Reset the paused ticks
    mPausedTicks = 0;
  }
}

unsigned long Timer::getTicks()
{

  unsigned long time = 0;


  if (mStarted)
  {
  
    if (mPaused)
    {

      time = mPausedTicks;
    }
    else
    {
    
      time = millis() - mStartTicks;
    }
  }

  return time;
}

bool Timer::isStarted()
{

  return mStarted;
}

bool Timer::isPaused()
{

  return mPaused && mStarted;
}
















/*
A Wrapper class for the Distance sensor. 
Allows for easily getting the distance and just figuring out when the robot should back up.

*/
class DistanceSensor
{
public:
  DistanceSensor();
  //Get the distance that the distance sensor is seeing in front of the robot.
  float getDistance();
  //a simple boolean to tell if the robot is too close to something.
  bool willHit();




};

DistanceSensor::DistanceSensor()
{

}

float DistanceSensor::getDistance()
{
  return (float)distancesensor.ping_cm();
}

bool DistanceSensor::willHit()
{
  if (distancesensor.ping_cm() < 5)
  {
    return true;
  }
  return false;
}

















/*
Class that handles the moter. It moves the robot in various directions, and stops it when needed.
*/
class Motor
{
public:
  Motor();
  //Moves the robot right.
  void moveRight();
  //Moves the robot left.
  void moveLeft();
  //Moves the robot forward.
  void moveForward();
  //Moves the robot backward.
  void moveBackward();
  //Stops all movement.
  void Stop();





};


Motor::Motor()
{


}

void Motor::moveRight()
{
  //move right

}

void Motor::moveLeft()
{
  //moveLeft

}

void Motor::moveForward()
{
  //move forward
  Serial.begin(9600);
  pinMode(rightPol, OUTPUT);
  pinMode(leftPol, OUTPUT);
}

void Motor::moveBackward()
{
  //moveBackwards

}

void Motor::Stop()
{
  //Stop;

}












//State abstract class to work with the state machine.
class State
{
public:
  //Generic move function that moves the robot whichever direction it needs to go.
  virtual void move(Motor * robot, DistanceSensor sensor) = 0;

};












//State that moves the robot to the left.
class moveLeft : public State
{
public:
  moveLeft();
  void move(Motor * robot, DistanceSensor sensor);
  


};


moveLeft::moveLeft()
{
  //nothing;
}


void moveLeft::move(Motor * robot, DistanceSensor sensor)
{
  if (!sensor.willHit())
  robot->moveLeft();
}












//State that moves the robot to the right.
class moveRight : public State
{
public:
  moveRight();
  void move(Motor * robot, DistanceSensor sensor);



};


moveRight::moveRight()
{
  //nothing;
}


void moveRight::move(Motor * robot, DistanceSensor sensor)
{
  if (!sensor.willHit())
  robot->moveRight();
}










//State that moves the robot forward.
class moveForward : public State
{
public:
  moveForward();
  void move(Motor * robot, DistanceSensor sensor);



};

moveForward::moveForward()
{
  //nothing;
}


void moveForward::move(Motor * robot, DistanceSensor sensor)
{
  if (!sensor.willHit())
    robot->moveForward();
  
}










//State that moves the robot backward.
class moveBackward : public State
{
public:
  moveBackward();
  void move(Motor * robot, DistanceSensor sensor);



};

moveBackward::moveBackward()
{
  //nothing;
}


void moveBackward::move(Motor * robot, DistanceSensor sensor)
{
  robot->moveBackward();
}













//State that allows the robot to move on it's own.
class autoMovement : public State
{
public:
  autoMovement();
  void move(Motor * robot, DistanceSensor sensor);
private:
  Timer timer;
  int direction;

};

autoMovement::autoMovement()
{
  
  //nothing;
}

//State that handles the automatic movement of the robot. A work in progress.
void autoMovement::move(Motor * robot, DistanceSensor sensor)
{
  //If we're not going to hit something, go forward.
  if (!sensor.willHit())
  {
  
    //If the timer is started (just got out of going backwards), and the 1 second has not passed since it's started, turn.
    if (timer.isStarted() && timer.getTicks() < 1000)
    {
      robot->moveLeft();
    }
    //if timer is started and 1 second HAS passed, we're done turning so lets just go forward now.
    else if (timer.isStarted())
    {
      timer.stop();
    }
    else
    robot->moveForward();
  }
  else
  {
    //if the timer hasnt started, start it.
    if (!timer.isStarted())
    {
      timer.start();
    }
    //If the timer is started, and the 1 second have not passed since it's started, go back.
    if (timer.isStarted() && timer.getTicks() < 1000)
    {
      robot->moveBackward();
    }
    //if timer is started and 1 seconds HAS passed, reset it to time for the turn.
    else if (timer.isStarted())
    {
      timer.start();
    }
  }
}











class Break : public State
{

public:
  Break();
  void move(Motor * robot, DistanceSensor sensor);
};

Break::Break()
{
  //nothing;
}


void Break::move(Motor * robot, DistanceSensor sensor)
{

  robot->Stop();
}




class LowerCrane : public State
{

public:
  LowerCrane();
  void move(Motor * robot, DistanceSensor sensor);
};

LowerCrane::LowerCrane()
{
  //nothing;
}


void LowerCrane::move(Motor * robot, DistanceSensor sensor)
{
  //lower crane.
}



class RaiseCrane : public State
{

public:
  RaiseCrane();
  void move(Motor * robot, DistanceSensor sensor);
};

RaiseCrane::RaiseCrane()
{
  //nothing;
}


void RaiseCrane::move(Motor * robot, DistanceSensor sensor)
{
  //raise crane.
}





//Pointer to the current state.
State * currentstate;


//Used to track current state. Since this method loops, repeated deleting and remaking the current state for the user holding down the button would not be good.
char currentAction;


/*
a method that manages the state machine depending on what the user inputted.
*/
void handleInput(char input)
{
  if (input != currentAction)
  {
    if (currentstate != nullptr)
    delete currentstate;


    switch (input)
    {
    case INPUT_FORWARDS:
      currentstate = new moveForward();
      break;
    case INPUT_BACKWARDS:
      currentstate = new moveBackward();
      break;
    case INPUT_LEFT:
      currentstate = new moveLeft();
      break;
    case INPUT_RIGHT:
      currentstate = new moveRight();
      break;
    case INPUT_STOP:
      currentstate = new Break();
      break;
    case INPUT_LOWER_CRANE:
      currentstate = new LowerCrane();
      break;
    case INPUT_RAISE_CRANE:
      currentstate = new RaiseCrane();
      break;
    case INPUT_AI:
      currentstate = new autoMovement();
      break;

    }
    currentAction = input;
  }
}
DistanceSensor sensor;
Motor * robot;
void setup()
{
  //Set up robot 
  robot = new Motor();
  Serial.begin(9600);
  //Declare pins
  pinMode(rightPol, OUTPUT);
  pinMode(leftPol, OUTPUT);
  pinMode(rightBrake, OUTPUT);
  pinMode(leftBrake, OUTPUT);
}

void loop()
{ //If theres input available, handle it.
  if (Serial.available() > 0) {
    char input = Serial.read();
    handleInput(input);
  

  }


  //Handle whatever movement we're currently doing.
  currentstate->move(robot, sensor);







}
