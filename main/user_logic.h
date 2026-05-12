#ifndef USER_LOGIC_H
#define USER_LOGIC_H

void run_user_sequence() {
  sensors.readAll();
  if (sensors.getValue(0) > Config::LINE_BLACK_THRESHOLD) {
    for (int _i = 0; _i < 3; _i++) {
      setMotors(150, 150);
      delay(1000);
      setMotors(150, 150);
    }
  } else {
    {
      int _base = 60;
      TickType_t _start = xTaskGetTickCount();
      while ((xTaskGetTickCount() - _start) * portTICK_PERIOD_MS < 3000) {
        sensors.readAll();
        long _ws = 0, _ts = 0;
        for (int i = 0; i < 6; i++) {
          uint16_t v = sensors.getValue(i);
          if (v > Config::LINE_BLACK_THRESHOLD) { _ws += v * Config::LINE_WEIGHTS[i]; _ts += v; }
        }
        if (_ts > 0) { float _p = (float)_ws / _ts; float _st = linePID.compute(0, _p); robot.steer((int)_st, _base); }
        else { robot.stop(); linePID.reset(); }
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      robot.stop();
    }
  }

}

#endif