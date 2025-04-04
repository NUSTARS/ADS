bool openFlapsAccel(float* accel_vals)
{
  if (accel_vals[2] < 0) //z
  {
    Serial.println("Burnout reached!");
    return true;
  }
  return false;
}

bool openFlapsHeight(float height)
{
   if (height > BURNOUT_HEIGHT) {
      Serial.println("Burnout height reached!");
      return true;
    } 
      return false;
}

bool burnoutReached(float* accel_vals, float height)
{
  if (openFlapsHeight(height) && openFlapsAccel(accel_vals)) {
      Serial.println("BURNOUT REACHED!");

      return true;
  }
  return false;

}


