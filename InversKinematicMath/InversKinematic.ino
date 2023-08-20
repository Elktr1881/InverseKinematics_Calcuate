#define PI 3.14159265358979323846
#define ShoulderLong 10
#define ElbowLong 10
#define WirstLong 10

float koordinateX,koordinateY,koordinateZ;

void mainOfKinemematics(float x, float y, float z)
{
    Serial.print("\n\n===================Inverse Kinematis 4Dof ARM Robot=====================\n\n");
    Serial.print("X = "); Serial.println(x);
    Serial.print("Y = "); Serial.println(y);
    Serial.print("Z = "); Serial.println(z);
    Serial.print("\n\n");

    //mencari koordinat x dan y setelah sudut base disesuaikan
    float xz,yz;
    xz = sqrt(pow(x,2)+pow(y,2));
    yz = z;
    Serial.print("XZ = "); Serial.println(xz);
    Serial.print("YZ = "); Serial.println(yz);
    Serial.print("\n");

    //mencari jarak dan sudut ke set point
    float spaceSetpoint = sqrt(pow(xz,2)+pow(yz,2));
    float setPointAngle = atan(yz/xz)*180/PI;
    Serial.print("Jarak set Pion = "); Serial.println(spaceSetpoint);
    Serial.print("sudut set Pion = "); Serial.println(setPointAngle);
    Serial.print("\n");

    if(spaceSetpoint < 24.2)
    {
        //mencari jarak dan sudut lengan Elbow
        float elbowAngle = 180;
        float spaceElbow = sqrt(pow(ShoulderLong,2)+pow(ElbowLong,2)-(2*ShoulderLong*ElbowLong*cos((elbowAngle-90)*PI/180)));
        Serial.print("Jarak Elbow = "); Serial.println(spaceElbow);
        Serial.print("sudut Elbow = "); Serial.println(elbowAngle);
        Serial.print("\n");

        //mencari sudut-sudut lengan Shoulder
        float ElbowWirstAngle = acos((pow(spaceElbow,2)+pow(spaceSetpoint,2)-pow(WirstLong,2))/(2*spaceElbow*spaceSetpoint))*180/PI;
        float shoulderAngle = setPointAngle + ElbowWirstAngle + (180-(elbowAngle-90))/2;
        Serial.print("Sudut antara Elbow & wirst = "); Serial.println(ElbowWirstAngle);
        Serial.print("sudut Shoulder = "); Serial.println(shoulderAngle);
        Serial.print("\n");

        //mencari arah dan sudut lengan Wirst
        float wirstVector = 180-shoulderAngle+(180-(elbowAngle-90))/2;
        float wirstX = cos(wirstVector*PI/180)*spaceElbow;
        float wirstY = sin(wirstVector*PI/180)*spaceElbow;
        float wirstAngle =180-shoulderAngle-(atan((wirstY-yz)/(wirstX+xz))*180/PI);

        Serial.print("Arah Wirst = "); Serial.println(wirstVector);
        Serial.print("XW = "); Serial.println(wirstX);
        Serial.print("YW = "); Serial.println(wirstY);
        Serial.print("sudut Wirst = "); Serial.println(wirstAngle);
        Serial.print("\n");
    }
    else
    {
        //mencari sudut lengan Shoulder
        float shoulderAngle = setPointAngle;
        if(shoulderAngle <= 0){shoulderAngle = 0;}
        Serial.print("sudut Shoulder = "); Serial.println(shoulderAngle);
        Serial.print("\n");

        //mencari sudut lengan Elbow
        float elbowAngle =  90+acos((pow(ShoulderLong,2)+pow(spaceSetpoint-WirstLong,2)-pow(ElbowLong,2))/(2*ShoulderLong*(spaceSetpoint-WirstLong)))*180/PI;
        Serial.print("sudut Elbow = "); Serial.println(elbowAngle);
        Serial.print("\n");
        
        //mencari sudut lengan Elbow
        float wirstAngle =  90-WirstLong+acos((pow(ShoulderLong,2)+pow(spaceSetpoint-WirstLong,2)-pow(ElbowLong,2))/(2*ShoulderLong*(spaceSetpoint-WirstLong)))*180/PI;
        Serial.print("sudut Wirst = "); Serial.println(wirstAngle);
        Serial.print("\n");
    }
}
