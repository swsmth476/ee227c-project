function reference_controller(block)

  setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of dialog parameters   
  block.NumDialogPrms = 0;

  %% Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = 6;
  block.InputPort(1).DirectFeedthrough = false;

  block.OutputPort(1).Dimensions       = 2;
  
  %% Set block sample time to inherited
  block.SampleTimes = [-1 0];
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
  
%endfunction

function InitConditions(block) 
  block.OutputPort(1).Data = [0; 0];
  
%endfunction

function Output(block)
  global mdl;

  % model predictive controller for reference model %
  
  z = block.InputPort(1).Data;
  
  % in order to fix weird initialization bug
  if(z == zeros(6,1))
      z = [mdl.z0; zeros(2,1)]; % set to initial state
  end
  
  % mid state cost %
  Q = blkdiag(0, 1, 0); % only care about vehicle spacing midway %
  Q_mid = mdl.H'*Q*mdl.H;
  q_mid = -mdl.wg'*Q*mdl.H;
  
  % final state cost %
  Q = blkdiag(1, 2.5, 1); % quadratic state cost
  Qf = mdl.H'*Q*mdl.H;
  qf = -mdl.wg'*Q*mdl.H;
  
  % input cost %
  R = zeros(2);
  r = zeros(1,2); % no input cost
  
  % constraints %
  h_delta = 35;
  hl = [1 0 -1 0]; % to get lead vehicle's separation
  hd = 50*3; % desired headway
  
  % acceleration input bounds
  a_upper = 8;
  a_lower = -8;
  
  % jerk bounds %
  j_upper = .1;
  j_lower = -.1;
  
  Hx = [hl zeros(1,2);
        -hl zeros(1,2);
        zeros(2,4) eye(2);
        zeros(2,4) -eye(2)];
  
  hx = [hd + h_delta;
        -hd + h_delta;
        a_upper;
        a_upper;
        -a_lower;
        -a_lower];

  Hu = [eye(2);
        -eye(2)];
    
  hu = [j_upper;
        j_upper;
        -j_lower;
        -j_lower];
  
  % use augmented system dynamics and T-step MPC %
  
  A = [[mdl.Fd mdl.Gd]; zeros(2,4) eye(2)];
  B = [zeros(4,2); eye(2)];
  theta = [mdl.theta_hatd; zeros(2,1)];

  % augmented cost weights (only for final state) %
  Q_aug = blkdiag(Q_mid, R);
  q_aug = [q_mid r];
  
  Qf_aug = blkdiag(Qf, R);
  qf_aug = [qf r];
  
  R_aug = eye(2);
  r_aug = zeros(1,2);
  
  % construct augmented system QP %
  [H, f, A_constr, b_constr] = make_QP(7, 5, 2, A, B, theta, z, Qf_aug, qf_aug', Q_aug, q_aug', R, r', Hx, hx, Hu, hu);
  
  [v, ~, exitflag] = quadprog(H, f, A_constr, b_constr);
  
  v = v(1:2) + z(5:6);

  if(exitflag ~= 1)
      error(['quadprog failed with exit flag ', num2str(exitflag)]);
  end
  
  block.OutputPort(1).Data = v;

%endfunction