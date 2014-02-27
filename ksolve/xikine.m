function q = xikine(T, sol)
  if nargin < 2; sol = ones(1, 3); end
  px = T(1,4); py = T(2,4); pz = T(3,4);
  if sol(1) == 1
    q(1) = angle(px+py*1i);
  else
    q(1) = angle(-px-py*1i);
  end
  S1 = sin(q(1));
  C1 = cos(q(1));

  if sol(2) == 1
    q(2) = angle(pz*2.0i-sqrt((pz*4.0-4.0)^2-(pz*-2.0+pz^2+C1^2*px^2+S1^2*py^2+C1*S1*px*py*2.0-4.0)^2+(C1*px*4.0+S1*py*4.0)^2)-pz^2*1i-C1^2*px^2*1i-S1^2*py^2*1i-C1*S1*px*py*2.0i+4.0i)-angle(pz*-4.0-C1*px*4.0i-S1*py*4.0i+4.0);
  else
    q(2) = -angle(pz*-4.0-C1*px*4.0i-S1*py*4.0i+4.0)+angle(pz*2.0i+sqrt((pz*4.0-4.0)^2-(pz*-2.0+pz^2+C1^2*px^2+S1^2*py^2+C1*S1*px*py*2.0-4.0)^2+(C1*px*4.0+S1*py*4.0)^2)-pz^2*1i-C1^2*px^2*1i-S1^2*py^2*1i-C1*S1*px*py*2.0i+4.0i);
  end
  S2 = sin(q(2));
  C2 = cos(q(2));

  if sol(3) == 1
    q(3) = angle(-sqrt((-S2+S2*pz+C1*C2*px+C2*S1*py-2.0)^2+(C2-C2*pz+C1*S2*px+S1*S2*py)^2-9.0)+3.0i)-angle(-C2-S2*1i+C2*pz+S2*pz*1i+C1*C2*px*1i-C1*S2*px+C2*S1*py*1i-S1*S2*py-2.0i);
  else
    q(3) = angle(sqrt((-S2+S2*pz+C1*C2*px+C2*S1*py-2.0)^2+(C2-C2*pz+C1*S2*px+S1*S2*py)^2-9.0)+3.0i)-angle(-C2-S2*1i+C2*pz+S2*pz*1i+C1*C2*px*1i-C1*S2*px+C2*S1*py*1i-S1*S2*py-2.0i);
  end
  S3 = sin(q(3));
  C3 = cos(q(3));

end
