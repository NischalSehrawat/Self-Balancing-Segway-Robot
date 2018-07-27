function dx = rob_sim(t,y,PRM)
  
  dx = (PRM.A - PRM.B_mot_torq*PRM.K)*y;
  
end

