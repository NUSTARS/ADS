#

bool openFlapsAccel(sensors_event_t* event)
{
  if(event->type == SENSOR_TYPE_LINEAR_ACCELERATION)
  {
    double accel = event->acceleration.z;
    if (accel < 0)
    {
      Serial.println("Burnout reached!");
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool openFlapsHeight(barometerData* baro)
{
   if ((baro->alt) > BURNOUT_HEIGHT) {
      Serial.println("Burnout height reached!");
      return true;
    } else {
      return false;
    }
}

bool openFlaps(sensors_event_t* event, barometerData* baro)
{
  if (openFlapsHeight(baro) && openFlapsAccel(event)) {
      SetDesiredAreaPercent(100);
      Serial.println("SETTING FLAPS TO 100!");

      return true;
  }
  return false;

}


