int main()
{
  Project proj; 
  
  string pathProject = readStringFromCin("# Path of a new project: \n"); 
  proj.load(pathProject); 
  

  calibrateCamera(objPoints, imgPoints, cmat, dvec, rvec, tvec); 
  proj.addCamera("LeftCam", cmat, dvec, rvec, tvec); 
  
  proj.addTargets("FixedForCameraCorrection", fixedPoints); 
  proj.
  
  
  
  
  
  
  
  
  
  
  
  
}
