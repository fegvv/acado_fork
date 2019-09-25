#!/bin/bash 

# note: has to run from acado_fork root dir
# edit DEPLOY_DIR to the full path of the tamp planning package dir
DEPLOY_DIR="/home/larsvens/ros/tamp__ws/src/saarti/saarti"



DIR_NAME="rtisqp_export"

# run the code generation executable
echo "running code generation"
./examples/code_generation/mpc_mhe/code_generation_rtisqp
sleep 0.5

# include qpoases package with the exported code
echo
echo "copying /external_packages/qpoases to " $DIR_NAME
cp -r external_packages/qpoases $DIR_NAME

echo "deploying generated code to:" $DEPLOY_DIR
# rm dir if existed prevoiously
echo "removing" $DEPLOY_DIR/$DIR_NAME
rm -r $DEPLOY_DIR/$DIR_NAME

# copy the new dir to the deploy location
echo "copying" $DIR_NAME "to" $DEPLOY_DIR
cp -r $DIR_NAME $DEPLOY_DIR

# rejoyce	
echo
echo "successfully deployed rtisqp solver to " $DEPLOY_DIR

