% Set up serial communications
s1 = serial('COM3', 'BaudRate', 9600, 'Parity', 'none', 'DataBits', 8, 'StopBits', 1, 'FlowControl', 'none');
fopen(s1);
x=1:1000;               % initiate the x-axis values
y = zeros(1000, 1);     % initiate the y-axis values
ylim([0 1000]);         % fix the y-axis
hLine = plot(x,y);
ylabel({'Light Intensity', '(in lx)'});     % label the y-axis
StripChart('Initialize', gca);      % initiate the strip chart

for i = 1:100          % run loop 100 times
    val = fscanf(s1);   % get a line of data from the serial
    results = sscanf(val, '%f,%f,%f,%f', [1,4]);    % parse line for values
    
    [r,c] = size(results);
    if c ~= 4    % check that data is valid (1x4 matrix)
       y(1:i) = prevVal;    % if not valid, update strip chart with the previous reading (to keep the chart responsive)
       StripChart('Update', hLine, y(1:i));
       xlabel({'Time', '(in 500ms)'});     % continuously update the x-axis label because StripChart overwrites
    else
        disp(results);  % display the data in the MATLAB console, if valid data
        y(1:i) = results(4);    % write the photocell value into y-matrix for the graph
        prevVal = results(4);   % store the most recent reading
        StripChart('Update', hLine, y(1:i));    % update the graph with new y-data
        xlabel({'Time', '(in 500ms)'});     % continuously update the x-axis label because StripChart overwrites
    end
end

fclose(s1);