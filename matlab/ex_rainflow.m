% The Signal Processing Toolbox of Matlab R2017b has a 'rainflow'
% command.  This are the examples from the online documentation,
% see <https://www.mathworks.com/help/signal/ref/rainflow.html>.
%
% Looks like MathWorks implemented rainflow cycle counting
% correctly.

clear;
close('all');

%%% Cycle Counts with Known Sample Rate

% Generate a signal that resembles a load history, consisting of
% sinusoid half-periods connecting known, equispaced reversals.
% The signal is sampled at 512 Hz for 8 seconds.
% Plot the extrema and the signal.
%
%      fs = 512;
%
%      X = [-2 1 -3 5 -1 3 -4 4 -2];
%      lX = length(X)-1;
%
%      Y = -diff(X)/2.*cos(pi*(0:1/fs:1-1/fs)') + (X(1:lX)+X(2:lX+1))/2;
%      Y = [Y(:);X(end)];
%
%      plot(0:lX,X,'o',0:1/fs:lX,Y)

fs = 512;

X = [-2 1 -3 5 -1 3 -4 4 -2];
lX = length(X)-1;

Y = bsxfun(@plus, bsxfun(@times, -diff(X)/2, cos(pi*(0:1/fs:1-1/fs)')), (X(1:lX)+X(2:lX+1))/2);
Y = [Y(:);X(end)];

plot(0:lX,X,'o',0:1/fs:lX,Y);

% Compute cycle counts for the data.  Display the matrix of cycle
% counts.
%
%      [c,hist,edges,rmm,idx] = rainflow(Y,fs);
%
%      T = array2table(c,'VariableNames',{'Count','Range','Mean','Start','End'})
%
% T =
%
%   7x5 table
%
%     Count    Range    Mean    Start    End
%     _____    _____    ____    _____    ___
%
%     0.5      3        -0.5    0        1
%     0.5      4          -1    1        2
%       1      4           1    4        5
%     0.5      8           1    2        3
%     0.5      9         0.5    3        6
%     0.5      8           0    6        7
%     0.5      6           1    7        8

c = cc4fat(Y, '-label', '-zero');
c(:, 4:5) = c(:, 4:5) ./ fs;

T = [c(:, 3), c(:, 1) .* 2, c(:, 2), min(c(:, 4:5), [], 2), max(c(:, 4:5), [], 2)]

% Matlab
%
% T =
%
%           0.5            3         -0.5            0            1
%           0.5            4           -1            1            2
%             1            4            1            4            5
%           0.5            8            1            2            3
%           0.5            9          0.5            3            6
%           0.5            8            0            6            7
%           0.5            6            1            7            8
%
%
% Octave
%
% T =
%
%          0.5           3        -0.5           0           1
%          0.5           4          -1           1           2
%            1           4           1           4           5
%          0.5           8           1           2           3
%          0.5           9         0.5           3           6
%          0.5           8           0           6           7
%          0.5           6           1           7           8

clear;
close('all');

%%% Cycle Counts with Known Time Values

% Generate a signal that resembles a load history, consisting
% of sinusoid half-periods connecting known, unevenly spaced
% reversals. The signal is sampled at 10 Hz for 15 seconds.
% Plot the extrema and the signal.
%
%      fs = 10;
%
%      X = [0 1 3 4 5 6 8 10 13 15];
%      Y = [-2 1 -3 5 -1 3 -4 4 -2 6];
%
%      Z = [];
%      for k = 1:length(Y)-1
%          x = X(k+1)-X(k);
%          z = -(Y(k+1)-Y(k))*cos(pi*(0:1/fs:x-1/fs)/x)+Y(k+1)+Y(k);
%          Z = [Z z/2];
%      end
%      Z = [Z Y(end)];
%
%      t = linspace(X(1),X(end),length(Z));
%      plot(X,Y,'o',t,Z)

fs = 10;

X = [0 1 3 4 5 6 8 10 13 15];
Y = [-2 1 -3 5 -1 3 -4 4 -2 6];

Z = [];
for k = 1:length(Y)-1
    x = X(k+1)-X(k);
    z = -(Y(k+1)-Y(k))*cos(pi*(0:1/fs:x-1/fs)/x)+Y(k+1)+Y(k);
    Z = [Z z/2];
end
Z = [Z Y(end)];

t = linspace(X(1),X(end),length(Z));
plot(X,Y,'o',t,Z);

% Compute cycle counts for the data. Display the matrix of cycle counts.
%
%      [c,hist,edges,rmm,idx] = rainflow(Z,t);
%
%      TT = array2table(c,'VariableNames',{'Count','Range','Mean','Start','End'})
%
% TT =
%
%   7x5 table
%
%     Count    Range    Mean    Start    End
%     _____    _____    ____    _____    ___
%
%     0.5       3       -0.5     0        1
%     0.5       4         -1     1        3
%       1       4          1     5        6
%     0.5       8          1     3        4
%       1       6          1    10       13
%     0.5       9        0.5     4        8
%     0.5      10          1     8       15

c = cc4fat(Z, t);

TT = [c(:, 3), c(:, 1) .* 2, c(:, 2), min(c(:, 4:5), [], 2), max(c(:, 4:5), [], 2)]

% Matlab
%
% TT =
%
%           0.5            3         -0.5            0            1
%           0.5            4           -1            1            3
%             1            4            1            5            6
%           0.5            8            1            3            4
%             1            6            1           10           13
%           0.5            9          0.5            4            8
%           0.5           10            1            8           15
%
%
% Octave
%
% TT =
%
%          0.5           3        -0.5           0           1
%          0.5           4          -1           1           3
%            1           4           1           5           6
%          0.5           8           1           3           4
%            1           6           1          10          13
%          0.5           9         0.5           4           8
%          0.5          10           1           8          15

clear;
close('all');

%%% Algorithms

% [...]
% Now collect the results.
%
%  Cycle Count | Range | Mean  | Start |  End
% -------------+-------+-------+-------+-------
%      1/2     |   3   | -0.5  |   A   |   B
%      1/2     |   4   | -1    |   B   |   C
%       1      |   4   |  1    |   E   |   F
%      1/2     |   8   |  1    |   C   |   D
%       1      |   3   | -0.5  |   K   |   L
%       1      |   1   |  2.5  |   M   |   N
%       1      |   7   |  0.5  |   H   |   J
%      1/2     |   9   |  0.5  |   D   |   G
%      1/2     |  10   |  1    |   G   |   P
%
% Compare this to the result of running rainflow on the sequence:
%
%      q = rainflow([-2 1 -3 5 -1 3 -4 4 -3 1 -2 3 2 6])
%
% q =
%
%     0.5000    3.0000   -0.5000    1.0000    2.0000
%     0.5000    4.0000   -1.0000    2.0000    3.0000
%     1.0000    4.0000    1.0000    5.0000    6.0000
%     0.5000    8.0000    1.0000    3.0000    4.0000
%     1.0000    3.0000   -0.5000   10.0000   11.0000
%     1.0000    1.0000    2.5000   12.0000   13.0000
%     1.0000    7.0000    0.5000    8.0000    9.0000
%     0.5000    9.0000    0.5000    4.0000    7.0000
%     0.5000   10.0000    1.0000    7.0000   14.0000

c = cc4fat([-2 1 -3 5 -1 3 -4 4 -3 1 -2 3 2 6], 'ABCDEFGHJKLMNP');

Count = c(:, 3);
Range = c(:, 1) .* 2;
Mean = c(:, 2);
Start = char(min(c(:, 4:5), [], 2));
End = char(max(c(:, 4:5), [], 2));

T = [num2cell(Count), num2cell(Range), num2cell(Mean), cellstr(Start), cellstr(End)]

% Matlab
%
% T =
%
%     [0.5]    [ 3]    [-0.5]    'A'    'B'
%     [0.5]    [ 4]    [  -1]    'B'    'C'
%     [  1]    [ 4]    [   1]    'E'    'F'
%     [0.5]    [ 8]    [   1]    'C'    'D'
%     [  1]    [ 3]    [-0.5]    'K'    'L'
%     [  1]    [ 1]    [ 2.5]    'M'    'N'
%     [  1]    [ 7]    [ 0.5]    'H'    'J'
%     [0.5]    [ 9]    [ 0.5]    'D'    'G'
%     [0.5]    [10]    [   1]    'G'    'P'
%
%
% Octave
%
% T =
% {
%   [1,1] =        0.5
%   [2,1] =        0.5
%   [3,1] =          1
%   [4,1] =        0.5
%   [5,1] =          1
%   [6,1] =          1
%   [7,1] =          1
%   [8,1] =        0.5
%   [9,1] =        0.5
%   [1,2] =          3
%   [2,2] =          4
%   [3,2] =          4
%   [4,2] =          8
%   [5,2] =          3
%   [6,2] =          1
%   [7,2] =          7
%   [8,2] =          9
%   [9,2] =         10
%   [1,3] =       -0.5
%   [2,3] =         -1
%   [3,3] =          1
%   [4,3] =          1
%   [5,3] =       -0.5
%   [6,3] =        2.5
%   [7,3] =        0.5
%   [8,3] =        0.5
%   [9,3] =          1
%   [1,4] = A
%   [2,4] = B
%   [3,4] = E
%   [4,4] = C
%   [5,4] = K
%   [6,4] = M
%   [7,4] = H
%   [8,4] = D
%   [9,4] = G
%   [1,5] = B
%   [2,5] = C
%   [3,5] = F
%   [4,5] = D
%   [5,5] = L
%   [6,5] = N
%   [7,5] = J
%   [8,5] = G
%   [9,5] = P
% }

clear;
close('all');

%% ex_rainflow.m ends here
