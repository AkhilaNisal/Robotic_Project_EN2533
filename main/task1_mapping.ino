#define CELL_COUNT 4

int nx = 0;
int ny = 0;

int dir = 0;

//0,1,2,3
int dx[4] = { 1, 0, -1, 0 };
int dy[4] = { 0, 1, 0, -1 };

int lookDir(int i) {
  if (dir + i > 3) return dir + i - 4;
  else if (dir + i < 0) return dir + i + 4;
  return dir + i;
}

int cellMap[CELL_COUNT][CELL_COUNT];

void pickObjectInLeft() {
}

void pickObjectInRight() {
}

int checkLeft() {
  //1 - obj
  //-1 - obs
  //0 - free
  return 0;
}

int checkRight() {
  //1 - obj
  //-1 - obs
  //0 - free
  return 0;
}

bool obj_found = false;
bool obj_search = true;

void onLineCross() {

  if (obj_search) {

    int res_left = checkLeft();
    int res_right = checkRight();

    if (res_left == 1) {
      if (obj_found) {
        // map
        obj_search = false;
        // cellMap[nx + dx[lookDir(1)]][ny + dy[lookDir(1)]] = 1;
      } else {
        obj_found = true;
        pickObjectInLeft();
      }
    }
    if (res_right == 1) {
      if (obj_found) {
        // map
        // cellMap[nx + dx[lookDir(-1)]][ny + dy[lookDir(-1)]] = 1;
        obj_search = false;
      } else {
        obj_found = true;
        // cellTraverse();
        pickObjectInRight();
      }
    }
  }

  nx += dx[dir];
  ny += dy[dir];
  Serial.print(dir);
  Serial.print(" ");
  Serial.print(nx);
  Serial.print(" ");
  Serial.println(ny);

  if (nx == CELL_COUNT - 1 && dir == 0) {
    dir = 1;
    moveForwardDist(260);
    turnLeft();
    moveForwardDist(-100);
    is_aligning = false;
    lt = -1;
    rt = -1;
    left_prev = 0;
    right_prev = 0;

    now_loc = -20000;
  } else if (nx == 0 && dir == 2) {
    dir = 1;
    moveForwardDist(260);
    turnRight();
    moveForwardDist(-100);
    is_aligning = false;
    lt = -1;
    rt = -1;
    left_prev = 0;
    right_prev = 0;

    now_loc = -20000;
  } else if (nx == CELL_COUNT - 1 && dir == 1 && ny % 2 == 0) {
    dir = 2;
    moveForwardDist(260);
    turnLeft();
    moveForwardDist(-100);
    is_aligning = false;
    lt = -1;
    rt = -1;
    left_prev = 0;
    right_prev = 0;

    now_loc = -20000;
  } else if (nx == 0 && ny != 0 && dir == 1 && ny % 2 == 0) {
    dir = 0;
    moveForwardDist(260);
    turnRight();
    moveForwardDist(-100);
    is_aligning = false;
    lt = -1;
    rt = -1;
    left_prev = 0;
    right_prev = 0;

    now_loc = -20000;
  }
}
