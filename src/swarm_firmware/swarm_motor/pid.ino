/* PID Helper Functions */ 

// Limits the value of *val to be minimum <= *val <= maximum
void limit(int *val, int minimum, int maximum) {
  if (*val > maximum) {
    *val = maximum;
  } else if (*val < minimum) {
    *val = minimum;
  }
}

int pid(int target, int current, int p, int i, int d, int i_clamp, int out_clamp, int punch, int deadzone, int *last, int *accumulate_error) {
  int error = target - current;

  // Deadzone stuff
  if (-deadzone < error && error < deadzone) {
    return 0;
  }

  // P stuff
  int p_term = p * error;

  // I stuff
  *accumulate_error += i * error;
  limit(accumulate_error, -i_clamp, i_clamp);
  int i_term = *accumulate_error * i;

  // D stuff
  int d_term  = (*last - current) * d;
  *last = current;

  // Punch stuff
  int punch_term = 0;
  if (error > 0) {
    punch_term = punch;
  } else if (error < 0) {
    punch_term = -punch;
  }

  int sum = p_term + i_term + d_term + punch_term;
  limit(&sum, -out_clamp, out_clamp);
  return sum;
}