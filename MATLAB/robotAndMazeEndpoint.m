%-------------------------------------------%
% BEGIN: function robotAndMazeEndpoint.m    %
%-------------------------------------------%
function output = robotAndMazeEndpoint(input)

if input.auxdata.primeDynamicsUsed
    q = input.phase.integral;
    output.objective = q;
else
    output.objective = input.phase(1).finaltime;
end


%-------------------------------------------%
% END: function brachistochroneEndpoint.m   %
%-------------------------------------------%

