function res = TransformOrientation(orient_vector, screw, angle)

    z           = dual_quater();
    z           = computeTransform(screw, angle); 
   
    res         = transform(orient_vector, z);
   
    