%% Example for MultiPolyRegress: Polynomial Fit for v, A, p, T
% Here, we aim to fit a 2nd-degree polynomial to 4 independent variables:
% v (velocity), a (angle of attack), p (pressure), and T (temperature).
% The goal is to find a good polynomial fit for these variables to predict
% the dependent variable (Y).

%% Load Data
% Load your dataset, ensuring the independent variables (v, A, p, T) and
% the dependent variable (Y) are appropriately structured.
data = readtable('data.csv'); % Assuming the file is named 'data.csv'

% Extract columns into individual variables
v = data.v;  % Velocity
a = data.a;  % angle of attach
p = data.p;  % Pressure
T = data.T;  % Temperature
Y = data.Y;  % Dependent variable

save('data.mat', 'v', 'a', 'p');

load data.mat % Replace with your dataset file

% v, A, p, T should be columns in X, and Y should be a separate vector.
% X = [v, A, p, T]; Y = your dependent variable.

%% Perform a Polynomial Fit
% Fit a 2nd-degree polynomial for all 4 variables.
reg = MultiPolyRegress(v, a, 2); % Perform regression with degree 2.

%% Normalization (Optional)
% Normalize the inputs and redefine error metrics, such as MAE, MAESTD,
% CVMAE, and CVMAESTD. This affects only error calculation and not the fit.
reg = MultiPolyRegress([v, a], Y, 2, 'range');

%% Visualize the Fit
% Generate a scatter plot of the fit versus the actual values.
reg = MultiPolyRegress([v, a], Y, 2, 'figure');

%% Polynomial Variable Constraints (Optional)
% Specify the maximum power for each variable individually.
% For instance, if you want to limit powers of v and T to 1, while allowing
% a and p to have powers up to 2:
reg = MultiPolyRegress([v, a], Y, 2, [1 2 2 1 2]); 
PolynomialFormula = reg.PolynomialExpression;

%% Evaluate a New Data Point
% Evaluate the polynomial for a new data point (e.g., row 250 of X).
NewDataPoint = [v(250), a(250)];
NewScores = repmat(NewDataPoint, [length(reg.PowerMatrix), 1]).^reg.PowerMatrix;
EvalScores = ones(length(reg.PowerMatrix), 1);
for ii = 1:size(reg.PowerMatrix, 2)
    EvalScores = EvalScores .* NewScores(:, ii);
end
yhatNew = reg.Coefficients' * EvalScores; % Estimated Y for the new point.

%% View the Polynomial Expression
% Display the polynomial formula for the fitted model.
PolynomialFormula = reg.PolynomialExpression;

%% Goodness-of-Fit Measures
% Assess accuracy and compare models with different polynomial degrees.
for ii = 1:4
    reg = MultiPolyRegress([v, a], Y, ii); % Fit polynomials of degree 1 to 5.
    CVMAE(ii) = reg.CVMAE; % Collect cross-validated mean absolute errors.
end
CVMAE % Display error metrics for comparison.


