function niryo_realtime_ik()
% NIRYO_REALTIME_IK
% Connect to CoppeliaSim, stream a dummy pose, compute IK to reach dummy
% (using your provided analytic candidate-generation + wrist-solver),
% and send the best joint set to the robot at a limited rate.
%
% Requirements: remApi.m in MATLAB path, CoppeliaSim scene playing with
%   simRemoteApi.start(<PORT>) running. Adjust PORT and dummy name below.

%% ---------------- Config ----------------
PORT = 19991;               % change if your scene uses different port
dummyCandidates = {'dummy','/NiryoOne/dummy','NiryoOne/dummy','NiryoOne_dummy','dummy#0'};
solveInterval = 0.20;       % seconds between IK solves (rate-limiter)
printPose = true;           % print dummy pose each loop
maxSolveTime = 1.5;         % seconds allowed for solver try (soft)

%% -------------- connect ------------------
sim = remApi('remoteApi');
sim.simxFinish(-1);
clientID = sim.simxStart('127.0.0.1', PORT, true, true, 5000, 5);
if clientID < 0
    sim.delete();
    error('Cannot connect to CoppeliaSim on port %d. Start remote API server.', PORT);
end
fprintf('Connected to CoppeliaSim on port %d (clientID=%d)\n', PORT, clientID);

%% -------------- find joint handles robustly --------------
baseVariants = { 'NiryoOneJoint', 'NiryoOne_joint', 'NiryoOneJoint_', 'NiryoOne::joint', 'NiryoOne_joint_' };
foundHandles = nan(1,6);
for i = 1:6
    found = false;
    triedNames = {};
    for v = 1:numel(baseVariants)
        namesToTry = {
            sprintf('%s%d', baseVariants{v}, i), ...
            sprintf('%s%d#0', baseVariants{v}, i), ...
            sprintf('%s_%d', baseVariants{v}, i), ...
            sprintf('%s_joint%d', baseVariants{v}, i), ...
            sprintf('%s%d_joint', baseVariants{v}, i)
        };
        for t = 1:numel(namesToTry)
            name = namesToTry{t};
            triedNames{end+1} = name; %#ok<AGROW>
            [rc, handle] = sim.simxGetObjectHandle(clientID, name, sim.simx_opmode_blocking);
            fprintf('Try get handle "%s" -> rc=%d\n', name, rc);
            if rc == sim.simx_return_ok
                fprintf('  Found joint %d as "%s" (handle=%d)\n', i, name, handle);
                foundHandles(i) = handle;
                found = true;
                break;
            end
            pause(0.01);
        end
        if found, break; end
    end
    if ~found
        sim.simxFinish(clientID); sim.delete();
        fprintf('Tried names for joint %d:\n', i); disp(triedNames');
        error('Could not retrieve handle for joint %d. Copy exact object name from scene.', i);
    end
end

%% -------------- find dummy handle --------------
hDummy = -1;
for k = 1:numel(dummyCandidates)
    [rc, h] = sim.simxGetObjectHandle(clientID, dummyCandidates{k}, sim.simx_opmode_blocking);
    fprintf('Try get handle "%s" -> rc=%d\n', dummyCandidates{k}, rc);
    if rc == sim.simx_return_ok
        hDummy = h;
        fprintf('  Found dummy as "%s" (handle=%d)\n', dummyCandidates{k}, hDummy);
        break;
    end
    pause(0.01);
end
if hDummy == -1
    sim.simxFinish(clientID); sim.delete();
    error('Could not find dummy. Copy exact object name from the scene and add to dummyCandidates.');
end

% start streaming pose (first call starts streaming)
sim.simxGetObjectPosition(clientID, hDummy, -1, sim.simx_opmode_streaming);
sim.simxGetObjectOrientation(clientID, hDummy, -1, sim.simx_opmode_streaming);
pause(0.05);

%% -------------- constants / limits used by IK --------------
k2 = 0.132; k3 = 0.342; k4 = 0.372;
k5 = 0.2317; k6 = 0.1033; k7 = 0.00748;

planar_link2 = k3 - k2;
planar_link3 = sqrt((k4 - k3)^2 + k5^2);

limits_deg = [ -175,  175;   % theta1
               -90,   36.7;  % theta2
               -80,   90;    % theta3
               -175,  175;   % theta4
               -100,  110;   % theta5
               -147.5,  147.5 ]; % theta6
limits_rad = deg2rad(limits_deg);

% initial warm-start guess for full theta
theta_guess = deg2rad([10; 10; 10; 10; 10; 10]);

%% -------------- main loop --------------
lastSolveT = tic;
fprintf('Entering main loop - move the dummy in CoppeliaSim to update target.\n');
while sim.simxGetConnectionId(clientID) ~= -1
    % read latest dummy pose from buffer
    [rcp, pos] = sim.simxGetObjectPosition(clientID, hDummy, -1, sim.simx_opmode_buffer);
    [rco, ori] = sim.simxGetObjectOrientation(clientID, hDummy, -1, sim.simx_opmode_buffer);

    if rcp ~= sim.simx_return_ok || rco ~= sim.simx_return_ok
        % if buffer not yet available, just wait shortly
        pause(0.02);
        continue;
    end

    if printPose
        fprintf('dummy pos=[%.4f %.4f %.4f]  ori=[%.4f %.4f %.4f]\n', pos(1),pos(2),pos(3), ori(1),ori(2),ori(3));
    end

    % rate-limit IK solves to avoid overload
    if toc_in_seconds(lastSolveT) < solveInterval
        pause(0.005); continue;
    end
    lastSolveT = tic_seconds();

    % build T_des using intrinsic XYZ Euler eul = [a b g]
    Rdes = eulerXYZ_to_R(ori);
    Tdes = [Rdes, pos(:); 0 0 0 1];

    % run the candidate-based IK pipeline using Tdes
    try
        t_start = tic;
        best_final = run_candidate_ik(Tdes, theta_guess, k2,k3,k4,k5,k6,k7, planar_link2, planar_link3, limits_rad);
        elapsed = toc(t_start);
        if elapsed > maxSolveTime
            warning('IK solve took %.3f s (over maxSolveTime %.3f). Skipping send this cycle.', elapsed, maxSolveTime);
            continue;
        end
        if isempty(best_final) || any(isnan(best_final))
            warning('IK failed — resetting guess and skipping this cycle.');
            theta_guess = deg2rad([0 0 0 0 0 0]);   % neutral pose instead of previous
            
            continue;
        end

        % send best_final to CoppeliaSim (oneshot)
        for j = 1:6
            sim.simxSetJointTargetPosition(clientID, foundHandles(j), best_final(j), sim.simx_opmode_oneshot);
        end
        theta_guess = best_final(:);  % warm-start next iter

    catch ME
        %warning('IK cycle failed: %s', ME.message);
        warning('IK cycle failed:');
        % keep previous guess, continue
    end

    % small pause to avoid tight loop
    pause(0.01);
end

% cleanup
sim.simxFinish(clientID);
sim.delete();
fprintf('Connection closed, exiting.\n');
end

%% ---------------- Utility & IK pipeline functions ----------------

function sec = tic_seconds()
    sec = tic;
end
function elapsed = toc_in_seconds(tic_val)
    % if tic_val is negative or empty -> large number
    if isempty(tic_val) || ~isnumeric(tic_val)
        elapsed = inf;
        return;
    end
    % interpret tic_val as tic() token; return elapsed seconds
    elapsed = toc(tic_val);
end

function best_final = run_candidate_ik(T_des, theta_guess, k2,k3,k4,k5,k6,k7, planar_link2, planar_link3, limits_rad)
    % This is the merged IK pipeline from your code, but using T_des (from dummy).
    % Returns best_final as 1x6 vector of joint angles (radians) or [] if none found.

    % Setup
    R_des = T_des(1:3,1:3);
    p_des = T_des(1:3,4);

    % compute wrist global target from T_des (replace previous Ttotal usage)
    wrist_global_x = -k6 * T_des(1,2) - k7 * T_des(1,3) + T_des(1,4);
    wrist_global_y = -k6 * T_des(2,2) - k7 * T_des(2,3) + T_des(2,4);
    wrist_global_z = -k6 * T_des(3,2) - k7 * T_des(3,3) + T_des(3,4);

    global_x = -0.100; global_y = -0.631; global_z = 0.051;
    px = wrist_global_x - global_x;
    py = wrist_global_y - global_y;
    pz = wrist_global_z - global_z;

    % theta1 candidates
    theta1_a = atan2(-px, py);
    theta1_b = wrapToPi(theta1_a + pi);
    theta1_list = [theta1_a, theta1_b];

    % wrist initial guess (from theta_guess)
    t4_guess = theta_guess(4);
    t5_guess = theta_guess(5);
    t6_guess = theta_guess(6);

    candidates = []; % columns: theta1..theta6 | pos_err | ori_err | total_err

    for idx = 1:2
        theta1 = theta1_list(idx);

        % rotate wrist coordinates into joint-2 frame
        px_dot = px * cos(theta1) + py * sin(theta1);
        py_dot = -px * sin(theta1) + py * cos(theta1);
        pz_dot = pz - k2;

        % compute theta3 and theta2 candidates
        [theta3_1, theta3_2, theta2_22, theta2_44] = compute_theta3(px_dot, py_dot, pz_dot, planar_link2, planar_link3);

        theta3_options = [theta3_1, theta3_2];
        theta2_options = [theta2_44, theta2_22];

        for j = 1:2
            t3_geom = theta3_options(j);
            t2_geom = theta2_options(j);
            if isempty(t2_geom) || isempty(t3_geom), continue; end

            if j == 1
                t3_deg = 80 + rad2deg(t3_geom);
                t2_deg = 90 - rad2deg(t2_geom);
            else
                t3_deg = 80 - rad2deg(t3_geom);
                t2_deg = -90 + rad2deg(t2_geom);
            end

            t1_candidate = theta1;
            t2_candidate = deg2rad(t2_deg);
            t3_candidate = deg2rad(t3_deg);

            theta_candidate = [t1_candidate, t2_candidate, t3_candidate, t4_guess, t5_guess, t6_guess];

            % joint limits
            in_limits = all(theta_candidate >= (limits_rad(:,1)'-1e-9) & theta_candidate <= (limits_rad(:,2)'+1e-9));
            if ~in_limits, continue; end

            [T_cand, ~, ~, ~] = compute_finalA(theta_candidate);

            Re = eye(3);
            pe = [0; k5 + k6; k4 + k7];
            Te = [Re, pe; 0 0 0 1];
            T_cand_total = T_cand * Te;

            p_c = T_cand_total(1:3,4);
            pos_err = norm(p_c - p_des);

            R_c = T_cand_total(1:3,1:3);
            ori_err = angleBetweenRot(R_c, R_des);

            w_pos = 1.0; w_ori = 0.5;
            total_err = w_pos * pos_err + w_ori * ori_err;

            candidates = [candidates; theta_candidate, pos_err, ori_err, total_err]; %#ok<AGROW>
        end
    end

    if isempty(candidates)
        best_final = [];
        return;
    end

    [~, order] = sort(candidates(:, end));
    candidates_sorted = candidates(order, :);
    best = candidates_sorted(1,:);
    best_theta = best(1:6);

    % compute wrist joint possibilities for that best shoulder/elbow (use T_des)
    shoulder_elbow_angles = [best_theta(1), best_theta(2), best_theta(3)];
    [t4_1,t4_2, t5_1, t5_2, t6_1, t6_2] = compute_wrist_joints(shoulder_elbow_angles, T_des);

    final_candidates = [
        best_theta(1), best_theta(2), best_theta(3), t4_1, t5_1, t6_1;
        best_theta(1), best_theta(2), best_theta(3), t4_1, t5_1, t6_2;
        best_theta(1), best_theta(2), best_theta(3), t4_1, t5_2, t6_1;
        best_theta(1), best_theta(2), best_theta(3), t4_1, t5_2, t6_2;
        best_theta(1), best_theta(2), best_theta(3), t4_2, t5_1, t6_1;
        best_theta(1), best_theta(2), best_theta(3), t4_2, t5_1, t6_2;
        best_theta(1), best_theta(2), best_theta(3), t4_2, t5_2, t6_1;
        best_theta(1), best_theta(2), best_theta(3), t4_2, t5_2, t6_2;
    ];

    % evaluate final candidates
    results = zeros(size(final_candidates,1), 4);
    for i = 1:size(final_candidates,1)
        cand = final_candidates(i,:);
        if any(cand < (limits_rad(:,1)'-1e-9)) || any(cand > (limits_rad(:,2)'+1e-9))
            pos_err = Inf; ori_err = Inf; total_err = Inf;
        else
            [T_cand, ~, ~, ~] = compute_finalA(cand);
            Re = eye(3);
            pe = [0; k5 + k6; k4 + k7];
            Te = [Re, pe; 0 0 0 1];
            T_cand_total = T_cand * Te;

            p_c = T_cand_total(1:3,4);
            pos_err = norm(p_c - p_des);
            R_c = T_cand_total(1:3,1:3);
            ori_err = angleBetweenRot(R_c, R_des);
            total_err = 1.0 * pos_err + 0.5 * ori_err;
        end
        results(i,:) = [pos_err, ori_err, total_err, i];
    end

    [~, ord_final] = sort(results(:,3));
    final_candidates_sorted = final_candidates(ord_final, :);

    % pick best valid candidate
    if isempty(final_candidates_sorted)
        best_final = [];
    else
        best_final = final_candidates_sorted(1,:);
    end
end

%% --- utility local functions (same as in your original script) ---
function y = wrapToPi(x)
    y = mod(x + pi, 2*pi) - pi;
end

function [Tfinal, Ttotal, pos, x] = compute_finalA(theta)
    t1 = theta(1); t2 = theta(2); t3 = theta(3);
    t4 = theta(4); t5 = theta(5); t6 = theta(6);

    c1 = cos(t1); s1 = sin(t1);
    c2 = cos(t2); s2 = sin(t2);
    c3 = cos(t3); s3 = sin(t3);
    c4 = cos(t4); s4 = sin(t4);
    c5 = cos(t5); s5 = sin(t5);
    c6 = cos(t6); s6 = sin(t6);

    k2 = 0.132; k3 = 0.342; k4 = 0.372; k5 = 0.2317; k6 = 0.1033; k7 = 0.00748;

    A0 = [ 1, 0, 0, -0.100;
           0, 1, 0, -0.631;
           0, 0, 1,   0.051;
           0, 0, 0,   1 ];

    A1 = [ c1, -s1, 0, 0;
           s1,  c1, 0, 0;
           0,   0,  1, 0;
           0,   0,  0, 1 ];

    A2 = [ 1,   0,   0,            0;
           0,  c2, -s2,       k2*s2;
           0,  s2,  c2, k2*(1 - c2);
           0,   0,   0,            1 ];

    A3 = [ 1,   0,   0,            0;
           0,  c3, -s3,       k3*s3;
           0,  s3,  c3, k3*(1 - c3);
           0,   0,   0,            1 ];

    A4 = [ c4,  0,  s4,      -k4*s4;
            0,  1,   0,          0;
         -s4,  0,  c4, k4*(1 - c4);
            0,  0,   0,          1 ];

    A5 = [ 1,  0,   0,                            0;
           0, c5, -s5, k5*(1 - c5) + k4*s5;
           0, s5,  c5,-k5*s5 + k4*(1 - c5);
           0,  0,   0,                            1 ];

    A6 = [ c6,  0,  s6,      -k4*s6;
            0,  1,   0,          0;
         -s6,  0,  c6, k4*(1 - c6);
            0,  0,   0,          1 ];

    Tfinal = A0 * A1 * A2 * A3 * A4 * A5 * A6;
    pos = Tfinal*[0;0;0;1];

    Re = eye(3);
    pe = [0; k5 + k6; k4 + k7];
    Te = [Re, pe; 0 0 0 1];
    Ttotal = Tfinal * Te;

    wrist_transformation = A4 * A5 * A6;
    wrist_position = [0; k5; k4; 1];
    x = wrist_transformation * wrist_position;
end

function [theta3_1, theta3_2, theta2_22, theta2_44] = compute_theta3(px_dot, py_dot, pz_dot, planar_link2, planar_link3)
    numerator = px_dot.^2 + py_dot.^2 + pz_dot.^2 - planar_link2.^2 - planar_link3.^2;
    denumerator = 2 * planar_link2 .* planar_link3;
    D = numerator ./ denumerator;
    D_clamped = min(max(D, -1), 1);
    D_sine = sqrt( max(0, 1 - D_clamped.^2) );

    theta3_1 = atan2(-D_sine, D_clamped);
    theta3_2 = atan2( D_sine, D_clamped);

    r = sqrt(px_dot.^2 + py_dot.^2);
    theta2_22 = atan2(pz_dot, r) - atan2(planar_link3 .* sin(theta3_1), planar_link2 + planar_link3 .* cos(theta3_1));
    theta2_44 = atan2(pz_dot, r) - atan2(planar_link3 .* sin(theta3_2), planar_link2 + planar_link3 .* cos(theta3_2));
end

function ang = angleBetweenRot(R1, R2)
    Rerr = R1' * R2;
    tr = trace(Rerr);
    cosang = (tr - 1)/2;
    cosang = min(max(cosang, -1), 1);
    ang = acos(cosang);
end

function [theta4_1,theta4_2, theta5_1, theta5_2, theta6_1, theta6_2] = compute_wrist_joints(thetas, Tdes)
    k2 = 0.132; k3 = 0.342;
    t1 = thetas(1); t2 = thetas(2); t3 = thetas(3);
    c1 = cos(t1); s1 = sin(t1);
    c2 = cos(t2); s2 = sin(t2);
    c3 = cos(t3); s3 = sin(t3);

    A0 = [ 1, 0, 0, -0.100;
           0, 1, 0, -0.631;
           0, 0, 1,   0.051;
           0, 0, 0,   1 ];
    A1 = [ c1, -s1, 0, 0;
           s1,  c1, 0, 0;
           0,   0,  1, 0;
           0,   0,  0, 1 ];
    A2 = [ 1,   0,   0,            0;
           0,  c2, -s2,       k2*s2;
           0,  s2,  c2, k2*(1 - c2);
           0,   0,   0,            1 ];
    A3 = [ 1,   0,   0,            0;
           0,  c3, -s3,       k3*s3;
           0,  s3,  c3, k3*(1 - c3);
           0,   0,   0,            1 ];

    T034 = A3 \ (A2 \ (A1 \ (A0 \ Tdes) ));
    R = T034(1:3,1:3);

    sy = sqrt(R(1,2)^2 + R(3,2)^2);
    theta5_1 = atan2(sy, R(2,2));
    theta5_2 = atan2(-sy, R(2,2));

    if abs(sin(theta5_1)) > 1e-9
        theta4_1 = pi - atan2(-R(1,2)/sin(theta5_1), R(3,2)/sin(theta5_1));
        theta6_1 = atan2(R(2,1)/sin(theta5_1), -R(2,3)/sin(theta5_1));
    else
        theta4_1 = 0;
        theta6_1 = atan2(-R(1,3), R(1,1));
    end

    if abs(sin(theta5_2)) > 1e-9
        theta4_2 = pi - atan2(-R(1,2)/sin(theta5_2), R(3,2)/sin(theta5_2));
        theta6_2 = atan2(R(2,1)/sin(theta5_2), -R(2,3)/sin(theta5_2));
    else
        theta4_2 = 0;
        theta6_2 = atan2(-R(1,3), R(1,1));
    end
end

function R = eulerXYZ_to_R(eul)
    a = eul(1); b = eul(2); g = eul(3);
    Rx = @(x)[1 0 0; 0 cos(x) -sin(x); 0 sin(x) cos(x)];
    Ry = @(y)[cos(y) 0 sin(y); 0 1 0; -sin(y) 0 cos(y)];
    Rz = @(z)[cos(z) -sin(z) 0; sin(z) cos(z) 0; 0 0 1];
    R = Rz(g) * Ry(b) * Rx(a);
end
