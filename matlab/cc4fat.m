% cc4fat -- cycle counting for fatigue analysis.
%
%      c = cc4fat(y, t, ...)
%
% Count the cycles in a signal history and return a cycle counting
% sequence.
%
% First argument Y is the signal history.  Value is a vector of
%  real numbers, logicals, or characters.
% Optional second argument T are the corresponding signal labels.
%  Value is a vector of real numbers, logicals, or characters.
%  If argument T is omitted or empty, don't label signal values
%  unless implicit labeling of signal values is enabled.
% The rest of the argument list are options.
%
% Return value C is a cycle counting sequence.  Each row in C is of
% the form [SA, SM, N] where SA is the signal amplitude, SM is the
% signal mean, and N is the cycle count.  If signal labeling is in
% effect, each row in C is of the form [SA, SM, N, T1, T2] where SA,
% SM, and N have the same meaning as before and T1 and T2 are the
% signal labels of the cycle's extrema values in chronological
% order.
%
% Options:
%
% -rainflow
%      Select the rainflow counting method.  This is the default
%      cycle counting method.
%
% -reservoir
%      Select the reservoir counting method.  Reservoir counting
%      creates the same result as rainflow counting iff the signal
%      history starts and ends with the absolute signal maximum.
%      Otherwise, reservoir counting is slightly more conservative
%      than rainflow counting.  Another property of reservoir
%      counting is that the resutling cycle counting sequence
%      only contains full cycles.
%
% -from-to, -range-mean, -amplitude-mean
%      Select the cycle representation.  Default is to use the
%      signal amplitude and signal mean (see above).  Range/mean
%      is like amplitude/mean except that the signal amplitude is
%      replaced by the signal range, i.e. two times the signal
%      amplitude.  With from/to cycle representation a cycle has
%      the form [S1, S2, N] where S1 and S2 are the extrema values
%      of the cycle in chronological order.
%
% -sign, -no-sign
%      Retain the sign of the signal amplitude or signal range.
%      Default is to utilize the absolute value for the cycle
%      representation.  This option has no effect for the from/to
%      cycle representation since the orientation of the cycle
%      is implied in the cycle representation.  If signed cycle
%      representation is enabled, the from/to signal values can
%      be calculated as S1=SM-SA and S2=SM+SA.
%
% -label, -no-label
%      Turn on implicit labeling of signal values, i.e. assign a
%      one-based index to the signal values.  This option has no
%      effect if the explicit signal labels argument T is supplied
%      and non-empty.  Contrary to that, the '-no-label' option
%      always disables signal labeling even if argument T is
%      supplied.
%
% -zero, -one
%      Use a zero-based index as implicit signal labels.  Default
%      is to use a one-based index.
%
% -no-merge, -merge
%      Don't merge similar consecutive cycles.  Default is to
%      merge similar consecutive cycles by adding the individual
%      cycle counts.  This optimization reduces the length of the
%      cycle counting sequence without loosing any information.
%      Cycles are similar if the cycle representation and the
%      optional signal labels are equal.
%
% -sort, -no-sort
%      Sort cycles by amplitude and mean value in descending
%      order.  This option has no effect if from/to cycle
%      representation is enabled.
%
% -half, -full
%      Express the cycle count N as the number of half cycles.
%      Default is to count the number of full cycles.
%
% -time, -no-time
%      Interpret signal labels as time values.  With that, each
%      cycle is of the form [SA, SM, N, T, P] where SA, SM, and N
%      have the usual meaning, T is the start time of the cycle,
%      and P is the period of one full cycle.
%
% -first, -third
%      Place the cycle count at the beginning of a cycle, i.e. each
%      cycle has the form [N, SA, SM, ...].  Default is the third
%      element.
%
% -transpose, -no-transpose
%      Transpose the resulting cycle counting sequence, i.e. each
%      cycle is a column vector instead of a row vector.
%
%
% Examples:
%
%      % Build MEX-file.
%      mex cc4fat.c rs-rainflow.c rs-matrix-transpose.c
%
%      % Create signal history.
%      y = round((rand(1E+5, 1) - 0.5) .* 100);
%      % Count cycles (raw mode).
%      c = cc4fat(y, '-no-merge');
%      % Sort cycles in descending order of
%      % load amplitude and mean load.
%      c = sortrows(c, [-1, -2]);
%      % Working column-wise is more efficient
%      % for the following code.
%      c = transpose(c);
%      % Merge similar cycles.
%      for k = fliplr(2:size(c, 2))
%        if c(1:2, k - 1) == c(1:2, k)
%          % Increment cycle count.
%          c(3, k - 1) = c(3, k - 1) + c(3, k);
%          % Clear merged cycle.
%          c(3, k) = 0;
%        end
%      end
%      % Round up remaining half cycles.
%      c(3, :) = ceil(c(3, :));
%      % Remove null cycles.
%      c(:, c(3, :) == 0) = [];
%      % Undo transposition.
%      c = transpose(c);

%% cc4fat.m --- the preceding comment is the documentation string.

% Copyright (C) 2011 Ralph Schleicher

% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions
% are met:
%
%    * Redistributions of source code must retain the above copyright
%      notice, this list of conditions and the following disclaimer.
%
%    * Redistributions in binary form must reproduce the above copyright
%      notice, this list of conditions and the following disclaimer in
%      the documentation and/or other materials provided with the
%      distribution.
%
%    * Neither the name of the copyright holder nor the names of its
%      contributors may be used to endorse or promote products derived
%      from this software without specific prior written permission.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
% FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
% COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
% BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
% LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
% ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.

%% cc4fat.m ends here
