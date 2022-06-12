data_2 = load("demofile2.txt");
time = data_2(:,1);
x_l = data_2(:,2);
x_r = data_2(:,3);

scatter(time, x_r)
ylim([-12000 -4000])
xlim([0 20])
xlabel("time (s)")
ylabel("encoder pulses")