lt = 1:1:20

plot(lt, error(1,:), '-r', lt, error(2,:), '-b');

legend('k Nearest-neighbor: k = 1', 'Connectivity-averaged');
xlabel('epoch');
ylabel('mean error ratio');

