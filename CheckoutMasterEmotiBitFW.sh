eval $(ssh-agent -s)
ssh-add ~/.ssh/id_rsa
cd ../EmotiBit_FeatherWing
git checkout master
cd ../EmotiBit_BMI160
git checkout master
cd ../EmotiBit_MAX30101
git checkout master
cd ../EmotiBit_MLX90632
git checkout master
cd ../EmotiBit_NCP5623
git checkout master
cd ../EmotiBit_SI7013
git checkout master
cd ../EmotiBit_XPlat_Utils
git checkout master
cd ../EmotiBit_External_EEPROM
git checkout master
cd ../EmotiBit_ADS1X15
git checkout master