/* PID Helper Functions */ 

// Limits the value of *val to be minimum <= *val <= maximum
void limit(float *val, int minimum, int maximum) {
  if (*val > maximum) {
    *val = maximum;
  } else if (*val < minimum) {
    *val = minimum;
  }
}

float pid(float target, float current, float p, float i, float d, float i_clamp, float out_clamp, float punch, float deadzone, float *last, float *accumulate_error) {
  float error = target - current;

  // Deadzone stuff
  if (-deadzone < error && error < deadzone) {
    return 0;
  }

  // P stuff
  float p_term = p * error;

  // I stuff
  *accumulate_error += i * error;
  limit(accumulate_error, -i_clamp, i_clamp);
  float i_term = *accumulate_error * i;

  // D stuff
  float d_term  = (*last - current) * d;
  *last = current;

  // Punch stuff
  float punch_term = 0;
  if (error > 0) {
    punch_term = punch;
  } else if (error < 0) {
    punch_term = -punch;
  }

  float sum = p_term + i_term + d_term + punch_term;
  limit(&sum, -out_clamp, out_clamp);
  return sum;
}
