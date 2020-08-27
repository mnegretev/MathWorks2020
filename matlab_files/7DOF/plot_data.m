t           = out.tout;
for i=1:7
  for j=1:length(t)
    desired_q(j)  = out.desired_q.signals.values(i,1,j);
    desired_qp(j) = out.desired_qp.signals.values(i,1,j);
  end
  figure
  hold on
  plot(t, out.measured_q.signals.values(:,i), 'Color', [0.00,0.00,1.00], 'LineWidth',3)
  plot(t, out.SMO_q.signals.values(:,i),      'Color', [0.25,0.75,0.95], 'LineWidth',3)
  plot(t, desired_q,                          'Color', [1.00,0.00,0.00], 'LineWidth',3)
  plot(t, out.EKF_q.signals.values(:,i)     , 'Color', [0.00,0.50,0.00], 'LineWidth',3)
  xlabel('Time [s]', 'Fontsize', 15)
  ylabel(strcat('Position $q_',int2str(i),'$ [rad]'), 'interpreter','latex', 'Fontsize',20)
  legend({'Measured','SMO Estimated','Goal','EKF Filtered'}, 'Location','southeast', 'Fontsize',13)

  figure
  hold on
  plot(t, out.SMO_speeds.signals.values(:,i), 'Color', [0.25,0.75,0.95], 'LineWidth',3)
  plot(t, out.real_qp.signals.values(:,i)   , 'Color', [0.00,0.00,1.00], 'LineWidth',3)
  plot(t, desired_qp                        , 'Color', [0.00,0.50,0.00], 'LineWidth',3)
  plot(t, out.EKF_speeds.signals.values(:,i), 'Color', [1.00,0.00,0.00], 'LineWidth',3)
  xlabel('Time [s]', 'Fontsize', 15)
  ylabel(strcat('Speed $\dot{q}_',int2str(i),'$ [rad/s]'), 'interpreter','latex', 'Fontsize', 20)
  legend({'SMO Estimated','Real Speed','Goal','EKF Estimated'}, 'Fontsize',13)
end
