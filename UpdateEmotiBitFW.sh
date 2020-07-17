eval $(ssh-agent -s)
ssh-add ~/.ssh/id_rsa
cd ../EmotiBit_FW_FeatherWing
git pull
cd ../EmotiBit_BMI160
git pull
cd ../EmotiBit_MAX30101
git pull
cd ../EmotiBit_MLX90632
git pull
cd ../EmotiBit_NCP5623
git pull
cd ../EmotiBit_SI7013
git pull
cd ../EmotiBit_XPlat_Utils
git pull