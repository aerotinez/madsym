close('all'); clear; clc;
setmadsympath();
load("poles_lagrange");
load("poles_gibbs_appell");

% xg = sort(poles_lagrange(:));
% xl = sort(poles_gibbs_appell(:));

% Initialize sorted matrix
xg = zeros(size(poles_lagrange));
xg(1, :) = sort(poles_lagrange(1, :)); % Sort the first row (lexicographically)

% Loop through rows to sort by minimizing complex distance
for i = 2:size(poles_lagrange, 1)
    prevRow = xg(i-1, :); % Previous row (sorted)
    currentRow = poles_lagrange(i, :); % Current row (unsorted)

    % Create a distance matrix for complex values
    distanceMatrix = abs(real(prevRow(:)) - real(currentRow(:).')) + ...
                 abs(imag(prevRow(:)) - imag(currentRow(:).')); % Euclidean distance in complex plane

    % Solve the assignment problem using the Hungarian algorithm
    [assignmentMatrix, ~] = munkres(distanceMatrix);
    [~, assignment] = max(assignmentMatrix, [], 2);

    % Assign sorted values based on the optimal assignment
    xg(i, :) = currentRow(assignment);
end

figure();
hold("on");
cv = linspace(0,1,size(xg,1));
c = jet(size(xg,1));
for k = 1:8
    scatter(real(xg(:,k)),imag(xg(:,k)),50,c,'filled');
end
% k = 8;
% scatter(real(xg(:,k)),imag(xg(:,k)),50,c,'filled');
% scatter(real(xl),imag(xl),'filled','SizeData',10);
hold("off");
fig = gcf;
axe = gca;
% set(axe,'Color','k')
box("on");
grid("on");
% axe.GridColor = ones(1,3);
xlim([-255,10]);
ylim([-63,63]);
xlabel("Re");
ylabel("Im");
sgrid(gca);
% leg = legend("Lagrange","Gibbs-Appell","Location","northwest");
% set(leg,'color',ones(1,3));
cb = colorbar;
cb.Label.String = "V_{x} (km/hr)";
clim([0,220]);
colormap(jet);
title("Sharp model Nyquist response");
fig.Position = [360,240,480,180];
saveas(fig,"C:\Users\marti\PhD\Articles\CDC24\Presentation\Figures\nyquist.eps",'epsc');