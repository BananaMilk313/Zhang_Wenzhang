function path = improved_A_Star(start, goal2, goal1)
    
    %startNode = startNode(:)'; 
    %goalNode = goalNode(:)'; 


    path(:, 1) = linspace(start(1), goal2, 10)';
    path(:, 2) = linspace(start(2), goal1, 10)';


%{
        % 获取维度（适用于 2D 或 3D）
    dims = length(startNode);
    
    % 逐维度计算插值
    path = zeros(10, dims);
    for i = 1:dims
        path(:, i) = linspace(startNode(i), goalNode(i), 10)'; % 这里转置确保输出是列向量
    end
%}
    % Start timing
    %tic;
    % Call the improved A* algorithm
    %path = improvedAStarAlgorithm(grid, startNode, goalNode);
    % End timing
    %elapsedTime = toc;
    %fprintf('improved_A* runtime: %.4f seconds\n', elapsedTime);

    %{
    % Display the path
    if ~isempty(path)
        plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2);
    else
        disp('No path found!');
    end
    %}
end

%{
function path = improvedAStarAlgorithm(grid, startNode, goalNode)
    % Initialization
    gridSize = size(grid);
    openList = [];
    closedList = false(gridSize);
    parent = zeros([gridSize, 2]); % Save parent nodes
    gCost = inf(gridSize);
    fCost = inf(gridSize);

    % Heuristic function: use Euclidean distance
    heuristic = @(node1, node2) sqrt((node1(1) - node2(1))^2 + (node1(2) - node2(2))^2);

    % Set weight α
    alpha = 0.5;

    % Initialize the start point
    gCost(startNode(1), startNode(2)) = 0;
    fCost(startNode(1), startNode(2)) = heuristic(startNode, goalNode);
    openList = [startNode, fCost(startNode(1), startNode(2))];

    % Main search loop
    while ~isempty(openList)
        % Find the node with the smallest f value in openList
        [~, idx] = min(openList(:,3));
        currentNode = openList(idx, 1:2);
        openList(idx, :) = []; % Remove the current node

        % If the goal node is reached, construct the path
        if isequal(currentNode, goalNode)
            path = reconstructPath(parent, startNode, goalNode);
            return;
        end

        % Add the current node to closedList
        closedList(currentNode(1), currentNode(2)) = true;
        % Visualize the closed nodes
        % Visualization operations should be commented out for runtime measurement
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plot(currentNode(1), currentNode(2), 'r.', 'MarkerSize', 15);
        pause(0.05); % Add a pause for observation
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % Get the neighbors of the current node
        neighbors = getNeighbors(currentNode, gridSize);

        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);

            % Skip obstacles or nodes already in closedList
            if grid(neighbor(1), neighbor(2)) == 0 || closedList(neighbor(1), neighbor(2))
                continue;
            end

            % Calculate temporary g value
            tentativeGCost = gCost(currentNode(1), currentNode(2)) + norm(neighbor - currentNode);

            % If the neighbor is not in openList or has a lower g value, update it
            if tentativeGCost < gCost(neighbor(1), neighbor(2))
                parent(neighbor(1), neighbor(2), :) = currentNode;
                gCost(neighbor(1), neighbor(2)) = tentativeGCost;

                % Add improved heuristic function
                if any(parent(currentNode(1), currentNode(2), :))
                    parentNode = squeeze(parent(currentNode(1), currentNode(2), :))';
                    hParent = heuristic(parentNode, goalNode);
                else
                    hParent = 0;
                end
                fCost(neighbor(1), neighbor(2)) = tentativeGCost + heuristic(neighbor, goalNode) + alpha * hParent;

                % Add to openList
                if ~any(ismember(openList(:,1:2), neighbor, 'rows'))
                    openList = [openList; neighbor, fCost(neighbor(1), neighbor(2))];
                    % Visualize the open nodes
                    % Visualization operations should be commented out for runtime measurement
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    plot(neighbor(1), neighbor(2), 'g.', 'MarkerSize', 15);
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                end
            end
        end
    end

    % If the search ends without finding a path
    path = [];
end

function neighbors = getNeighbors(node, gridSize)
    % Get neighbors in all directions (up, down, left, right, and diagonals)
    directions = [0, 1; 1, 0; 0, -1; -1, 0; -1, -1; -1, 1; 1, -1; 1, 1];
    neighbors = node + directions;

    % Keep only neighbors within bounds
    neighbors = neighbors(all(neighbors > 0, 2) & neighbors(:,1) <= gridSize(1) & neighbors(:,2) <= gridSize(2), :);
end

function path = reconstructPath(parent, startNode, goalNode)
    % Reconstruct the path from the parent node array
    path = goalNode;
    current = goalNode;
    while ~isequal(current, startNode)
        current = squeeze(parent(current(1), current(2), :))';
        path = [current; path];
    end
end
%}