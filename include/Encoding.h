#include <Config.h>

//alphabetic encoding
char* encode(int* flexion, int joyX, int joyY, bool joyClick, bool triggerButton, bool aButton, bool bButton, bool grab, bool pinch, bool calib, bool menu){
  static char stringToEncode[175];
  static char defaultEncoding[75];
  static char splayEncoding[50];
  static char strainEncoding[50];

  int trigger = (flexion[1] > ANALOG_MAX/2) ? (flexion[1] - ANALOG_MAX/2) * 2:0;

  sprintf(defaultEncoding, "A%dB%dC%dD%dE%dF%dG%dP%d%s%s%s%s%s%s%s%s", 
  flexion[0], flexion[1], flexion[2], flexion[3], flexion[4],
  joyX, joyY, trigger, joyClick?"H":"",
  triggerButton?"I":"", aButton?"J":"", bButton?"K":"", grab?"L":"", pinch?"M":"", menu?"N":"", calib?"O":""
  );

  #if USING_SPLAY
  sprintf(splayEncoding, "(AB)%d(BB)%d(CB)%d(DB)%d(EB)%d", 
  flexion[5], flexion[6], flexion[7], flexion[8], flexion[9]
  );
  #endif
  
  #if USING_STRAIN_GAUGE
  sprintf(strainEncoding, "(AC)%d(BC)%d(CC)%d(DC)%d(EC)%d", 
  flexion[10], flexion[11], flexion[12], flexion[13], flexion[14]
  );
  #endif
  
  sprintf(stringToEncode, "%s%s%s\n", defaultEncoding, splayEncoding, strainEncoding);
  return stringToEncode;
}

//legacy decoding
void decodeData(char* stringToDecode, int* hapticLimits){
  hapticLimits[0] = getArgument(stringToDecode, 'A'); //thumb
  hapticLimits[1] = getArgument(stringToDecode, 'B'); //index
  hapticLimits[2] = getArgument(stringToDecode, 'C'); //middle
  hapticLimits[3] = getArgument(stringToDecode, 'D'); //ring
  hapticLimits[4] = getArgument(stringToDecode, 'E'); //pinky
  //Serial.println("Haptic: "+ (String)hapticLimits[0] + " " + (String)hapticLimits[1] + " " + (String)hapticLimits[2] + " " + (String)hapticLimits[3] + " " + (String)hapticLimits[4] + " ");
}

int getArgument(char* stringToDecode, char command){
  char* start = strchr(stringToDecode, command);
  if (start == NULL)
    return -1;
  else
    return atoi(start + 1);
}