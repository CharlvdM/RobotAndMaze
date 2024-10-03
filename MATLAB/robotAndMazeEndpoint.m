%-------------------------------------------%
% BEGIN: function robotAndMazeEndpoint.m    %
%-------------------------------------------%
function output = robotAndMazeEndpoint(input)

% output.objective = input.phase(1).finaltime;

q = input.phase.integral;
output.objective = q;

%-------------------------------------------%
% END: function brachistochroneEndpoint.m   %
%-------------------------------------------%

