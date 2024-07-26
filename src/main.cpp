#include <SoftwareSerial.h>

#include <vector>

#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app_builder.h"

#define RX_IN 14
#define TX_OUT 12
#define LED_PIN 2

using namespace sensesp;

reactesp::ReactESP app;

EspSoftwareSerial::UART StlkSerial;

SKOutput<float>* compass_heading_output;
SKOutput<float>* pilot_course_output;
SKOutput<float>* depth_output;
SKOutput<float>* awa_output;
SKOutput<float>* aws_output;
SKOutput<float>* stw_output;
SKOutput<float>* cog_output;
SKOutput<float>* sog_output;

double depthBelowTransducer, speedOverGround, speedThroughWater,
    apparentWindSpeed;
double apparentWindAngle, courseOverGround, compassHeading, autoPilotCourse;

const float deg2rad = 0.0174532925;
const float knts2ms = 0.514444;

uint8_t checksum(char* s) {
  uint8_t c = 0;

  while (*s) c ^= *s++;

  return c;
}

char nmeaString[50];
char aBuffer[15];
char bBuffer[15];

void sendNMEA(char* mnmeaString) {
  Serial.print(nmeaString);
  Serial.print("*");
  uint8_t crc = checksum(nmeaString + 1);
  if (crc < 16) Serial.print("0");
  Serial.println(crc, HEX);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    ->set_hostname("sensesp-stlk-gateway")
                    ->get_app();

  StlkSerial.begin(4800, EspSoftwareSerial::SWSERIAL_8S1, RX_IN, TX_OUT, false,
                   95);

  pilot_course_output = new SKOutput<float>(
      "steering.autopilot.target.headingMagnetic", "/sensors/stlk/pilot_course",
      new SKMetadata("rad", "Pilot course"));

  compass_heading_output = new SKOutput<float>(
      "navigation.headingMagnetic", "/sensors/stlk/compass_heading",
      new SKMetadata("rad", "Vessel Compass heading"));

  depth_output = new SKOutput<float>(
      "environment.depth.belowTransducer", "/sensors/stlk/depth",
      new SKMetadata("m", "Depth under transducer"));

  awa_output =
      new SKOutput<float>("environment/wind/angleApparent", "/sensors/stlk/awa",
                          new SKMetadata("rad", "Apparent wind angle"));

  aws_output =
      new SKOutput<float>("environment/wind/speedApparent", "/sensors/stlk/aws",
                          new SKMetadata("m/s", "Apparent wind speed"));

  stw_output = new SKOutput<float>(
      "navigation/speedThroughWater", "/sensors/stlk/stw",
      new SKMetadata("m/s", "Vessel speed through the water"));

  sog_output =
      new SKOutput<float>("navigation/speedOverGround", "/sensors/stlk/sog",
                          new SKMetadata("m/s", "Vessel speed over ground"));

  cog_output = new SKOutput<float>(
      "navigation/headingTrue", "/sensors/stlk/cog",
      new SKMetadata("rad", "The current true north heading of the vessel"));

  sensesp_app->start();
}

void stlk_read() {
  static std::vector<uint8_t> message;

  while (StlkSerial.available()) {
    uint8_t inByte = StlkSerial.read();
    bool parity = StlkSerial.readParity();

    if (parity)  // Command byte received
    {
      message.clear();  // Clear previous message
      message.push_back(inByte);
    } else if (!message.empty())  // Data byte received
    {
      message.push_back(inByte);

      if (message[0] == 0x10 && message.size() == 4) {
        // Aparent wind angle
        apparentWindAngle = ((message[2] << 8) | (message[3])) / 2.0;

        // Serial.printf("Apparent Wind Angle: %.1f degrees\n",
        // apparentWindAngle);
        dtostrf(apparentWindAngle, 4, 2, aBuffer);
        dtostrf(apparentWindSpeed, 4, 2, bBuffer);
        snprintf(nmeaString, sizeof(nmeaString), "$--MWV,%s,R,%s,N,a", aBuffer,
                 bBuffer);
        sendNMEA(nmeaString);

        awa_output->set_input(apparentWindAngle * deg2rad);
      } else if (message[0] == 0x11 && message.size() == 4) {
        // Apparent wind speed
        apparentWindSpeed = ((message[2] & 0x7f) + (message[3] & 0x0f) / 10.0);

        // Serial.printf("Apparent Wind Speed: %.1f knots\n",
        // apparentWindSpeed);
        dtostrf(apparentWindSpeed, 4, 2, aBuffer);
        dtostrf(apparentWindAngle, 4, 2, bBuffer);
        snprintf(nmeaString, sizeof(nmeaString), "$--MWV,%s,R,%s,N,a", bBuffer,
                 aBuffer);
        sendNMEA(nmeaString);

        aws_output->set_input(apparentWindSpeed * knts2ms);
      }
      // Speed Through Water
      else if (message[0] == 0x20 && message.size() == 4) {
        speedThroughWater = ((message[3] << 8) | message[2]) / 10;

        // Serial.printf("Speed Through Water: %.1f knots\n",
        // speedThroughWater);
        dtostrf(speedThroughWater, 4, 2, aBuffer);
        dtostrf(compassHeading, 4, 2, bBuffer);
        snprintf(nmeaString, sizeof(nmeaString), "$--VHW,,T,%s,M,%s,N,,K",
                 bBuffer, aBuffer);
        sendNMEA(nmeaString);

        stw_output->set_input(speedThroughWater * knts2ms);
      }
      // Speed Over Ground
      else if (message[0] == 0x52 && message.size() == 4) {
        speedOverGround = ((message[3] << 8) | message[2]) / 10;

        Serial.printf("Speed Over Ground: %.1f knots\n", speedOverGround);

        sog_output->set_input(speedOverGround * knts2ms);
      }
      // Course Over Ground
      else if (message[0] == 0x53 && message.size() == 4) {
        uint8_t u = (message[1] & 0xf0) >> 4;
        uint8_t vw = message[2];
        courseOverGround =
            (u & 0x3) * 90 + (vw & 0x3F) * 2 + ((u & 0xC) >> 2) / 2;

        Serial.printf("Course Over Ground: %.1f degrees\n", courseOverGround);

        cog_output->set_input(courseOverGround * deg2rad);
      }
      // Depth Below Transducer
      else if (message[0] == 0x00 && message.size() == 5) {
        depthBelowTransducer = ((message[4] * 256 + message[3]) * 0.3077 / 10);

        // Serial.printf("Depth Below Transducer %.1f
        // Meters\n",depthBelowTransducer);
        dtostrf(depthBelowTransducer, 4, 2, aBuffer);
        snprintf(nmeaString, sizeof(nmeaString), "$--DBT,,f,%s,M,,F", aBuffer);
        sendNMEA(nmeaString);

        depth_output->set_input(depthBelowTransducer);
      } else if (message[0] == 0x84 && message.size() == 9) {
        uint8_t u = (message[1] & 0xf0) >> 4;
        uint8_t vw = message[2];

        // Get Compass Heading
        compassHeading = (u & 0x3) * 90 + (vw & 0x3F) * 2 +
                         (u & 0xC ? (u & 0xC == 0xC ? 2 : 1) : 0);
        Serial.printf("Auto Compass heading %.1f Degrees\n", compassHeading);

        compass_heading_output->set_input(compassHeading * deg2rad);
        // Get Turning Direction
        uint8_t dir = (u & 0x80) >> 3;

        Serial.printf("Auto Turning Direction %d \n", dir);

        // Get AP course
        uint8_t v = (message[2] & 0xc0) >> 6;
        uint8_t xy = message[3];

        autoPilotCourse = (v * 90) + (xy / 2);

        Serial.printf("Auto Pilot Course= %.1f Degrees \n", autoPilotCourse);
        pilot_course_output->set_input(autoPilotCourse * deg2rad);

        uint8_t z = message[4];
        if (z & 0x2 == 0) {
          Serial.println("standby");
        } else if (z & 0x2 == 2) {
          Serial.println("autom");
        } else if (z & 0x2 == 4) {
          Serial.println("vane");
        } else if (z & 0x8 == 8) {
          Serial.println("track");
        }
      }
    }
  }
}

void loop() {
  if (StlkSerial.available()) {
    stlk_read();
  }

  app.tick();
}
