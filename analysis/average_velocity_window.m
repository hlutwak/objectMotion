%% find average velocity of surrounding window

% first run full flow3d to get velocities

% % visualize positions of apertures and where deviation occured
figure (1), scatter(dots_deg(1,:), -dots_deg(2,:))
hold on, scatter(dots_deg(1,[dev0 dev1]), -dots_deg(2,[dev0 dev1]), 'r')

% find surrounding apertures based on dot density (avg degrees between apertures)
radius_window = 1/dot_density*7;
surround_window = [devPos(1)-radius_window devPos(1)+radius_window;
                   devPos(2)-radius_window devPos(2)+radius_window];
               
% window_idx = find(dots_deg(1,:)> surround_window(1) & dots_deg(1,:)< surround_window(3) ...
%                     & dots_deg(2,:)> surround_window(2) & dots_deg(2,:)< surround_window(4));  % find apertures that are in the window

% window_idx = find(dots_deg(1,:)> max(surround_window(1),0) & dots_deg(1,:)< surround_window(3) ...
%                     & dots_deg(2,:)> devPos(2) & dots_deg(2,:)< surround_window(4));  % find apertures that are in the window

% find based on radius
distance2d1 = vecnorm(devPos - dots_deg);
radius = 2; % in degrees
window_idx = find(distance2d1<radius);

window_idx = window_idx(window_idx~=dev1);

% %visualize surround
figure(1), hold on, scatter(dots_deg(1,window_idx), -dots_deg(2,window_idx), 30, [0.8500 0.3250 0.0980], 'filled')
axis equal
xlim([-20, 20])

%take average
avg_velocity = mean(velocity_field(:,window_idx),2);

%calculate average speed and speed of probe position without deviation in
%depth
devspeed = vecnorm(cond(1).steps(:,10));
avgspeed = vecnorm(avg_velocity);

% %histogram speed
% edges = 0:.2:3;
% figure(2), hold on, histogram(vecnorm(velocity_field(:,window_idx)),edges,'FaceColor', [.5, .5, .5])
% radius_count = histcounts(vecnorm(velocity_field(:,window_idx)), edges);
% % hold on, xline(devspeed, 'k');
% % hold on, xline(avgspeed, 'r');
% xlabel('speed deg/s')
% ylabel('count')

% display velocity
figure(3), 
hold on
quiver(dots_deg(1,:), -dots_deg(2,:), velocity_field(1,:), -velocity_field(2,:),  'Color', [.3 .3 .3],'LineWidth', 2,'AutoScale',0)

quiver(dots_deg(1,window_idx), -dots_deg(2,window_idx), velocity_field(1,window_idx), -velocity_field(2,window_idx),  'Color', [.7 .3 .3],'LineWidth', 2,'AutoScale',0)
axis equal
xlim([-20, 20])
ylim([-20, 20])


%% averages based on depth

% visualize depth map
if depth_structure == 2
   figure(4),imagesc(range_distance), colormap gray, colorbar
end

% visualize depth map of what is shown in the stimulus
figure(5), scatter3(dots_deg(1,:), dots_deg(2,:), dots_m(3,:))

% get average velocity from tree within receptive field
if depth_structure ==2
    tree_idx = find(dots_m(3,:)<10);
    background_idx = find(dots_m(3,:)>=10);
    tree_inwindow = intersect(tree_idx, window_idx);
    background_inwindow = intersect(background_idx, window_idx);
end
% %visualize tree in surround
% figure(3), hold on, quiver(dots_deg(1,tree_inwindow), -dots_deg(2,tree_inwindow), velocity_field(1,tree_inwindow), -velocity_field(2,tree_inwindow),  'Color', [.3 .7 .3],'LineWidth', 2,'AutoScale',0)
% axis equal
% xlim([-20, 20])

%calculate average speed and speed of probe position without deviation in
%depth
% avgspeed = vecnorm(avg_velocity);
% 
% avg_velocity = mean(velocity_field(:,tree_inwindow),2);

% % background vs foreground speed histogrram
% figure(2)
% hold on
% histogram(vecnorm(velocity_field(:,tree_inwindow)), edges, 'FaceColor', [.3, .7, .3])
% tree_count = histcounts(vecnorm(velocity_field(:,tree_inwindow)), edges);
% hold on
% histogram(vecnorm(velocity_field(:,background_inwindow)), edges, 'FaceColor', [.7, .3, .3])
% background_count = histcounts(vecnorm(velocity_field(:,background_inwindow)), edges);
% % 
% legend('total', 'tree', 'background')

%
hold on
grayd = distance2d1(window_idx)/max(distance2d1(window_idx));
c = [grayd' grayd' grayd'];
scatter(velocity_field(1,window_idx), -velocity_field(2,window_idx),50, c, 'filled');


%% display average on threshold figure
hold on, quiver(0, 0, avg_velocity(1), -avg_velocity(2), 'Color', [.3 .3 .3],'LineWidth', 2, 'LineStyle', '--','AutoScaleFactor',1)


%% finding covariance
% define size surround
radius = 2; % in degrees

% calculate distance
distance2d1 = vecnorm(devPos - dots_deg);
% get indices of surround
window_idx = find(distance2d1<radius);
window_idx = window_idx(window_idx~=dev1);

% weighted average based on
weighting_function = 'uniform';         %'inverse', 'gaussian', 'uniform'

switch weighting_function
    case 'inverse'
        weights = 1./distance2d1;
    case 'gaussian'
        sigma = 1/radius;
        weights = 1/((sigma^2)*2*pi).*exp(-((dots_deg(1,:)-devPos(1)).^2+(dots_deg(2,:)- devPos(2)).^2)./2*sigma^2);
    case 'uniform'
        weights = ones(size(dots_deg(1,:)));
end


w_avg = sum(weights(window_idx).*velocity_field(:,window_idx),2)/sum(weights(window_idx));

M_2 = (weights(window_idx).*velocity_field(:,window_idx)*velocity_field(:,window_idx)')./sum(weights(window_idx));

C = M_2 - w_avg*w_avg';

[V,D]=eig(C);
[d,ind] = sort(diag(D), 'descend');
D = D(ind,ind);
V = V(:,ind);
v1=V(:,1)*(D(1,1));%eigenvectors
v2=V(:,2)*(D(2,2));

theta = linspace(0, 2*pi, 50)';
x = [cos(theta) sin(theta)];
ellipse = 5*[v1 v2]*x';
addmean = ellipse+w_avg;

base_velocity = data(cond_idx(2)).steps(:,end,1);
basev_centered = ellipse+base_velocity;


%plot ellipse and mean
hold on
plot(addmean(1,:), -addmean(2,:), 'color', [.2 .2 .7], 'Linewidth', 2)
% hold on,
% plot(basev_centered(1,:), -basev_centered(2,:), 'color', [.2 .7 .7], 'Linewidth', 2)
hold on, scatter(w_avg(1), -w_avg(2), 50, [.2 .2 .7], 'filled')



