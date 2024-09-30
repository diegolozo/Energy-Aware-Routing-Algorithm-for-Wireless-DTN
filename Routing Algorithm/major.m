%% Energy Aware Routing Algorithm for Wireless Selfish DTN
% Conceptos Avanzados en Redes Inalámbricas Primavera 2024
% Diego Torreblanca Bravo
% Professor: Shaharyar Kamal
% Assistant Professor: Jorge Sandoval 
% FCFM - Universidad de Chile
% ----------------------------------------------------

%%
clear
close all
clc

%% Custom Simulation Parameters
% Custom parameters that can be modified by the user:
rng(1);                   % Random Seed 
n = 100;                  % Number of Nodes
buffer_size = int16(10);  % Buffer size for every node
iterations = 100;         % Number of cycles
num_connections = 4;      % Number of connections for each node
networ_change_rate = 20;  % Number of iterations to change network topolgy 
min_battery_level = 3;    % Valor mínimo de la batería
battery_range = 109;      % Rango de valores aleatorios de batería

%% Initial Simulation parameters
tic;
Node(n).batt = 100; % Battery level of every node
A = zeros(n); % Adjacency matrix
total_battery = 0; % Total Battery level of all nodes
Dead_node = 0; % Number of dead nodes
Loo = zeros(n); %appears to record the number of attempts to send packets 
% to each destination node, regardless of whether those attempts were 
% successful or not
Koo = zeros(n);  %appears to store the number of packets received 
% succesfully at each destination node
counter = 0;
packet_received = 0;

fprintf('Running simulation...\n');

%% Initialize properties of each node
for i = 1:n
    Node(i).batt = rand() * battery_range + min_battery_level; % Battery level
    total_battery = total_battery + Node(i).batt; 
    Node(i).no = i; % Node ID
    Node(i).buffer = zeros(buffer_size); % Buffer of valid nodes
    Node(i).start = 1; 
    Node(i).endd = 1;
    Node(i).sent = zeros(buffer_size);
    Node(i).dead = 0; % Dead or alive node indicator
    Node(i).buffval = 0;
end

%% Create connections between nodes and Graph

% n x num_connections matrix with random values from 1 to n
% For every node i, a maximum of num_connections nodes will be connected to it.
% Considering that every path has a weight between 0 and 10, 0 weigth
% meaning not connected.
K = randi(n, n, num_connections); 
L = randi(n, n, num_connections);

for i = 1:n
    for j = 1:num_connections
        A(round(K(i,j)), round(L(i,j))) = rand() * 10;
        A(round(L(i,j)), round(K(i,j))) = A(round(K(i,j)), round(L(i,j)));
    end
end

A(1:size(A,1)+1:end) = 0; % Delete self connections
S = sparse(A);
G = graph(S);
figure, plot(G)
%% Run iterations

waitbar_title = waitbar(0, 'Running iterations...');

for i = 1:iterations
    
    waitbar(i/iterations, waitbar_title, sprintf('Running iteration: %d/%d', i, iterations));

    STATISTICS.Sent(i+1) = packet_received; % Total of packet received until now
    STATISTICS.DEAD(i+1) = Dead_node; % Total of dead nodes until now
    STATISTICS.Battery(i+1) = total_battery; % Sum of battery level of all nodes until now

    % Select source and destination nodes randomly
    source = randi(n);
    dest = source;
    % Regenerate dest until it is different from source
    while dest == source
        dest = randi(n);
    end
    
    direct_path = 0; % Indicates if a direct path has been found between source and dest

    % Check if source node has enough battery
    if Node(source).batt > 1
        battery = Node(source).batt; % Save current battery level of source node
        Loo(dest) = Loo(dest) + 1;
        
        % Use shortestpath instead of graphshortestpath (old MATLAB versions)
        % Find shortest path between source and destination nodes
        [dist, ~] = shortestpath(G, source, dest);
        
        % Calculate sum of alpha and beta considering
        % distancia between source and dest nodes, and battery level of
        % source node
        obj_func = alpha(dist) + beta(battery);
        toforward = source;
        
        % Calculate sum of alpha and beta between source
        % node and every adjacent node to source, that is alive of course
        for j = 1:n
            batt_node_j = Node(j).batt;
            if A(source, j) > 0 && batt_node_j > 0
                [l, ~] = shortestpath(G, j, dest);
                current_obj_func = alpha(l) + beta(batt_node_j);
                if current_obj_func < obj_func
                    obj_func = current_obj_func;
                    toforward = j;
                end
            end
        end
        
        % If the best node to send the packet is exactly the destination
        % node from the source node, the packet is sent.
        if toforward == dest && source ~= dest
            packet_received = packet_received + 1;
            Koo(dest) = Koo(dest) + 1;
            direct_path = 1;
            counter = counter + 1;
        end

        % No direct path has been found between source and dest.
        % Source node will send the packet to an intermediate node
        if direct_path == 0

            % Reduce source node battery level in 0.3, and total battery 
            % level of all nodes as well
            Node(source).batt = Node(source).batt - 0.3;
            total_battery = total_battery - 0.3;
            
            % Checks if the buffer of toforward node is full
            if mod(Node(toforward).endd + 1, buffer_size) == Node(toforward).start
                Node(toforward).start = mod(Node(toforward).start + 1, buffer_size);
                Node(toforward).endd = mod(Node(toforward).endd + 1, buffer_size);
                if Node(toforward).endd == 0
                    Node(toforward).buffer(buffer_size) = dest;
                    Node(toforward).sent(buffer_size) = -1;    
                else
                    Node(toforward).buffer(Node(toforward).endd) = dest;
                    Node(toforward).sent(Node(toforward).endd) = -1;
                end
            else
                Node(toforward).endd = mod(Node(toforward).endd + 1, buffer_size);
                if Node(toforward).endd == 0
                    Node(toforward).buffer(buffer_size) = dest;
                    Node(toforward).sent(buffer_size) = -1;    
                else
                    Node(toforward).buffer(Node(toforward).endd) = dest;
                    Node(toforward).sent(Node(toforward).endd) = -1;
                end
            end
        end
    end

    for it = 1:n
        % Check if the node is not dead and has a battery level greater
        % than 20 units.
        if Node(it).dead == 0 && Node(it).batt > 20
            k = 1;
            while k <= buffer_size
                if k == 0
                    k = buffer_size;
                end
                % Checks if packet in position k has not been sent yet and
                % if there is a valid node to send it
                if Node(it).sent(k) == 0 && Node(it).buffer(k) ~= 0
                    direct_path = 0;
                    source = it;
                    dest = Node(it).buffer(k);
                    Node(it).sent(k) = 1;
                    Node(it).buffval = Node(it).buffval - 1;
                    
                    % Same calculation as before, calculating the sum of
                    % alpha and beta
                    [dist, ~] = shortestpath(G, source, dest);
                    battery = Node(source).batt;
                    obj_func = alpha(dist) + beta(battery);
                    toforward = dest;
                    
                    % Same calculation as before for every node, 
                    % calculating the sum of alpha and beta
                    for j = 1:n
                        if A(source, j) > 0
                            [l, ~] = shortestpath(G, j, dest);
                            current_obj_func = alpha(l) + beta(Node(j).batt);
                            if current_obj_func < obj_func
                                obj_func = current_obj_func;
                                toforward = j;
                            end
                        end
                    end
                    
                    % If the best node to send the packet is exactly the
                    % destination node from the source node, 
                    % the packet is sent.                   
                    if toforward == dest
                        direct_path = 1;
                        packet_received = packet_received + 1;
                        Koo(dest) = Koo(dest) + 1;
                    end
                    % No direct path has been found between source and dest.
                    % Source node will send the packet to an intermediate node
                    % It will keep doing this until the next node to
                    % forward the packet is the original destination.
                    if direct_path == 0
                        Node(source).batt = Node(source).batt - 0.3;
                        total_battery = total_battery - 0.3;
                        
                        if mod(Node(toforward).endd + 1, buffer_size) == Node(toforward).start
                            Node(toforward).start = mod(Node(toforward).start + 1, buffer_size);
                            Node(toforward).endd = mod(Node(toforward).endd + 1, buffer_size);
                            if Node(toforward).endd == 0
                                Node(toforward).buffer(buffer_size) = dest;
                                Node(toforward).sent(buffer_size) = -1;
                            else
                                Node(toforward).buffer(Node(toforward).endd) = dest;
                                Node(toforward).sent(Node(toforward).endd) = -1;
                            end
                        else
                            Node(toforward).endd = mod(Node(toforward).endd + 1, buffer_size);
                            if Node(toforward).endd == 0
                                Node(toforward).buffer(buffer_size) = dest;
                                Node(toforward).sent(buffer_size) = -1;
                            else
                                Node(toforward).buffer(Node(toforward).endd) = dest;
                                Node(toforward).sent(Node(toforward).endd) = -1;
                            end
                        end
                    end
                end
                k = k + 1;
            end
        end
    end

  %% Network Connectivity Modification
  % Every "networ_change_rate" iterations, the network will change
    if mod(i, networ_change_rate) == 0
        for kl = 1:n

            % Select source and destination nodes randomly
            source = randi(n);
            dest = source;
            % Regenerate dest until it is different from source
            while dest == source
                dest = randi(n);
            end
            
            if A(source, dest) == 0
                A(source, dest) = rand(1) * 10;
                A(dest, source) = A(source, dest);  % Make it simmetric
            else
                A(source, dest) = 0;
                A(dest, source) = 0; 
            end
        end
    end
    
    % Update the graph
    A(1:size(A,1)+1:end) = 0; % Delete self connections
    G = graph(A);
    
    %% Node State Management
    for it = 1:n
        % Check if a packet has not been sent
        for k = 1:buffer_size
            if Node(it).sent(k) == -1
                Node(it).sent(k) = 0;
                Node(it).buffval = Node(it).buffval + 1;
            end
        end
        % Check if the node is dead
        % If it is dead, it gets deleted from the graph along with its
        % packets
        if Node(it).batt < 1 && Node(it).dead == 0
            Dead_node = Dead_node + 1;
            Node(it).dead = 1;
            for k = 1:n
                A(it, k) = 0;
                A(k, it) = 0; 
            end
            for k = 1:buffer_size
                Node(it).sent(k) = 1;
            end
        end
    end
end
fprintf('Simulation completed!\n');
%% Plots
figure, plot(STATISTICS.Sent), title('Number of Packets Reached Destination'), xlabel({'Number of Iterations'}), ylabel({'Number of packets'});
figure, plot(STATISTICS.Battery), title('Total Battery'), xlabel({'Number of Iterations'}), ylabel({'Total Battery'});
figure, plot(STATISTICS.DEAD), title('Dead Nodes'), xlabel({'Number of Iterations'}), ylabel({'Number of Dead Nodes'});
figure, plot(G)
elapsedTime = toc;
fprintf('Elapsed time: %.2f seconds\n', elapsedTime);