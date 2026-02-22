long distToCounts(float dist) {
  return (long)((dist / (PI * physical.diameter)) * physical.encoderCPR);
}