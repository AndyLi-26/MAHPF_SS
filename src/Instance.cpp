#include"Instance.h"

int RANDOM_WALK_STEPS = 100000;

Instance::Instance(const string& map_fname, const string& agent_fname,
        int num_of_humans, int num_of_agents,
        int num_of_rows, int num_of_cols, int num_of_obstacles, int warehouse_width):
    map_fname(map_fname), agent_fname(agent_fname),
    num_of_humans(num_of_humans),num_of_agents(num_of_agents)
{
    bool succ = loadMap();
    if (!succ)
    {
        if (num_of_rows > 0 && num_of_cols > 0 && num_of_obstacles >= 0 &&
                num_of_obstacles < num_of_rows * num_of_cols) // generate random grid
        {
            generateConnectedRandomGrid(num_of_rows, num_of_cols, num_of_obstacles);
            saveMap();
        }
        else
        {
            cerr << "Map file " << map_fname << " not found." << endl;
            exit(-1);
        }
    }

    succ = loadAgents();

    if (!succ)
    {
        if (num_of_agents > 0)
        {
            generateRandomAgents(warehouse_width);
            // saveAgents();
            saveNathan();
        }
        else
        {
            cerr << "Agent file " << agent_fname << " not found." << endl;
            exit(-1);
        }
    }
    printAgents(AgentType::ROBOT);
    printAgents(AgentType::HUMAN);
    //bool temp=isConnected(start_human[0], goal_human[0]);
}

int Instance::randomWalk(int curr, int steps) const
{
    for (int walk = 0; walk < steps; walk++)
    {
        list<int> l = getNeighbors(curr);
        vector<int> next_locations(l.cbegin(), l.cend());
        auto rng = std::default_random_engine{};
        std::shuffle(std::begin(next_locations), std::end(next_locations), rng);
        for (int next : next_locations)
        {
            if (validMove(curr, next))
            {
                curr = next;
                break;
            }
        }
    }
    return curr;
}

void Instance::generateRandomAgents(int warehouse_width)
{
    cout << "Generate " << num_of_agents << " random start and goal locations " << endl;
    vector<bool> starts(map_size, false);
    vector<bool> goals(map_size, false);
    start_agent.resize(num_of_agents);
    goal_agent.resize(num_of_agents);

    if (warehouse_width == 0)//Generate agents randomly
    {
        // Choose random start locations
        int k = 0;
        while ( k < num_of_agents)
        {
            int x = rand() % num_of_rows, y = rand() % num_of_cols;
            int start = linearizeCoordinate(x, y);
            if (my_map[start] || starts[start])
                continue;

            // update start
            start_agent[k] = start;
            starts[start] = true;

            // find goal
            x = rand() % num_of_rows;
            y = rand() % num_of_cols;
            int goal = linearizeCoordinate(x, y);
            while (my_map[goal] || goals[goal])
            {
                x = rand() % num_of_rows;
                y = rand() % num_of_cols;
                goal = linearizeCoordinate(x, y);
            }
            //int goal = randomWalk(start, RANDOM_WALK_STEPS);
            //while (goals[goal])
            //	goal = randomWalk(goal, 1);

            //update goal
            goal_agent[k] = goal;
            goals[goal] = true;

            k++;
        }
    }
    else //Generate agents for warehouse scenario
    {
        // Choose random start locations
        int k = 0;
        while (k < num_of_agents)
        {
            int x = rand() % num_of_rows, y = rand() % warehouse_width;
            if (k % 2 == 0)
                y = num_of_cols - y - 1;
            int start = linearizeCoordinate(x, y);
            if (starts[start])
                continue;
            // update start
            start_agent[k] = start;
            starts[start] = true;

            k++;
        }
        // Choose random goal locations
        k = 0;
        while (k < num_of_agents)
        {
            int x = rand() % num_of_rows, y = rand() % warehouse_width;
            if (k % 2 == 1)
                y = num_of_cols - y - 1;
            int goal = linearizeCoordinate(x, y);
            if (goals[goal])
                continue;
            // update goal
            goal_agent[k] = goal;
            goals[goal] = true;
            k++;
        }
    }
}


bool Instance::addObstacle(int obstacle)
{
    if (my_map[obstacle])
        return false;
    my_map[obstacle] = true;
    int obstacle_x = getRowCoordinate(obstacle);
    int obstacle_y = getColCoordinate(obstacle);
    int x[4] = { obstacle_x, obstacle_x + 1, obstacle_x, obstacle_x - 1 };
    int y[4] = { obstacle_y - 1, obstacle_y, obstacle_y + 1, obstacle_y };
    int start = 0;
    int goal = 1;
    while (start < 3 && goal < 4)
    {
        if (x[start] < 0 || x[start] >= num_of_rows || y[start] < 0 || y[start] >= num_of_cols
                || my_map[linearizeCoordinate(x[start], y[start])])
            start++;
        else if (goal <= start)
            goal = start + 1;
        else if (x[goal] < 0 || x[goal] >= num_of_rows || y[goal] < 0 || y[goal] >= num_of_cols
                || my_map[linearizeCoordinate(x[goal], y[goal])])
            goal++;
        else if (isConnected(linearizeCoordinate(x[start], y[start]), linearizeCoordinate(x[goal], y[goal]))) // cannot find a path from start to goal
        {
            start = goal;
            goal++;
        }
        else
        {
            my_map[obstacle] = false;
            return false;
        }
    }
    return true;
}

bool Instance::isConnected(int start, int goal)
{
    std::queue<int> open;
    vector<bool> closed(map_size, false);
    open.push(start);
    //cout<<"s: "<<start<<endl;
    closed[start] = true;
    while (!open.empty())
    {
        int curr = open.front(); open.pop();
        //cout<<"poping: "<<curr<<" "<<my_map[curr]<<endl;
        if (curr == goal)
            return true;
        for (int next : getNeighbors(curr))
        {
            //cout<<"neighbors"<<next<<endl;
            if (closed[next])
                continue;
            open.push(next);
            closed[next] = true;
        }
    }
    return false;
}

void Instance::generateConnectedRandomGrid(int rows, int cols, int obstacles)
{
    cout << "Generate a " << rows << " x " << cols << " grid with " << obstacles << " obstacles. " << endl;
    int i, j;
    num_of_rows = rows + 2;
    num_of_cols = cols + 2;
    map_size = num_of_rows * num_of_cols;
    my_map.resize(map_size, false);
    // Possible moves [WAIT, NORTH, EAST, SOUTH, WEST]
    /*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
      moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
      moves_offset[Instance::valid_moves_t::EAST] = 1;
      moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
      moves_offset[Instance::valid_moves_t::WEST] = -1;*/

    // add padding
    i = 0;
    for (j = 0; j<num_of_cols; j++)
        my_map[linearizeCoordinate(i, j)] = true;
    i = num_of_rows - 1;
    for (j = 0; j<num_of_cols; j++)
        my_map[linearizeCoordinate(i, j)] = true;
    j = 0;
    for (i = 0; i<num_of_rows; i++)
        my_map[linearizeCoordinate(i, j)] = true;
    j = num_of_cols - 1;
    for (i = 0; i<num_of_rows; i++)
        my_map[linearizeCoordinate(i, j)] = true;

    // add obstacles uniformly at random
    i = 0;
    while (i < obstacles)
    {
        int loc = rand() % map_size;
        if (addObstacle(loc))
        {
            printMap();
            i++;
        }
    }
}

bool Instance::loadMap()
{
    using namespace boost;
    using namespace std;
    ifstream myfile(map_fname.c_str());
    if (!myfile.is_open())
        return false;
    string line;
    tokenizer< char_separator<char> >::iterator beg;
    getline(myfile, line);
    if (line[0] == 't') // Nathan's benchmark
    {
        char_separator<char> sep(" ");
        getline(myfile, line);
        tokenizer< char_separator<char> > tok(line, sep);
        beg = tok.begin();
        beg++;
        num_of_rows = atoi((*beg).c_str()); // read number of rows
        getline(myfile, line);
        tokenizer< char_separator<char> > tok2(line, sep);
        beg = tok2.begin();
        beg++;
        num_of_cols = atoi((*beg).c_str()); // read number of cols
        getline(myfile, line); // skip "map"
    }
    else // my benchmark
    {
        char_separator<char> sep(",");
        tokenizer< char_separator<char> > tok(line, sep);
        beg = tok.begin();
        num_of_rows = atoi((*beg).c_str()); // read number of rows
        beg++;
        num_of_cols = atoi((*beg).c_str()); // read number of cols
    }
    map_size = num_of_cols * num_of_rows;
    my_map.resize(map_size, false);
    // read map (and start/goal locations)
    for (int i = 0; i < num_of_rows; i++) {
        getline(myfile, line);
        for (int j = 0; j < num_of_cols; j++) {
            my_map[linearizeCoordinate(i, j)] = (line[j] != '.');
        }
    }
    myfile.close();

    // initialize moves_offset array
    /*moves_offset[Instance::valid_moves_t::WAIT_MOVE] = 0;
      moves_offset[Instance::valid_moves_t::NORTH] = -num_of_cols;
      moves_offset[Instance::valid_moves_t::EAST] = 1;
      moves_offset[Instance::valid_moves_t::SOUTH] = num_of_cols;
      moves_offset[Instance::valid_moves_t::WEST] = -1;*/
    return true;
}


void Instance::printMap() const
{
    for (int i = 0; i< num_of_rows; i++)
    {
        for (int j = 0; j < num_of_cols; j++)
        {
            if (this->my_map[linearizeCoordinate(i, j)])
                cout << '@';
            else
                cout << '.';
        }
        cout << endl;
    }
}


void Instance::saveMap() const
{
    ofstream myfile;
    myfile.open(map_fname);
    if (!myfile.is_open())
    {
        cout << "Fail to save the map to " << map_fname << endl;
        return;
    }
    myfile << num_of_rows << "," << num_of_cols << endl;
    for (int i = 0; i < num_of_rows; i++)
    {
        for (int j = 0; j < num_of_cols; j++)
        {
            if (my_map[linearizeCoordinate(i, j)])
                myfile << "@";
            else
                myfile << ".";
        }
        myfile << endl;
    }
    myfile.close();
}


bool Instance::loadAgents()
{
    using namespace std;
    using namespace boost;
    cout<<"loading agents"<<endl;

    string line;
    ifstream myfile (agent_fname.c_str());
    if (!myfile.is_open())
        return false;

    getline(myfile, line);
    if (line[0] == 'v') // Nathan's benchmark
    {
        if (num_of_agents == 0)
        {
            cerr << "The number of agents should be larger than 0" << endl;
            exit(-1);
        }
        start_agent.resize(num_of_agents);
        goal_agent.resize(num_of_agents);
        char_separator<char> sep("\t");

        for (int i = 0; i < num_of_agents; i++)
        {
            getline(myfile, line);
            tokenizer< char_separator<char> > tok(line, sep);
            task t=line2task(tok);
            start_agent[i]=t.first;
            goal_agent[i]=t.second;
        }

        start_human.resize(num_of_humans);
        goal_human.resize(num_of_humans);
        for (int i = 0; i < num_of_humans; i++)
        {
            getline(myfile, line);
            tokenizer< char_separator<char> > tok(line, sep);
            task t=line2task(tok);
            start_human[i]=t.first;
            goal_human[i]=t.second;
        }
    }
    else // My benchmark
    {
        char_separator<char> sep(",");
        tokenizer< char_separator<char> > tok(line, sep);
        tokenizer< char_separator<char> >::iterator beg = tok.begin();
        num_of_agents = atoi((*beg).c_str());
        start_agent.resize(num_of_agents);
        goal_agent.resize(num_of_agents);
        for (int i = 0; i<num_of_agents; i++)
        {
            getline(myfile, line);
            tokenizer< char_separator<char> > col_tok(line, sep);
            tokenizer< char_separator<char> >::iterator c_beg = col_tok.begin();
            pair<int, int> curr_pair;
            // read start [row,col] for agent i
            int row = atoi((*c_beg).c_str());
            c_beg++;
            int col = atoi((*c_beg).c_str());
            start_agent[i] = linearizeCoordinate(row, col);
            // read goal [row,col] for agent i
            c_beg++;
            row = atoi((*c_beg).c_str());
            c_beg++;
            col = atoi((*c_beg).c_str());
            goal_agent[i] = linearizeCoordinate(row, col);
        }
    }
    myfile.close();
    return true;
}


task Instance::line2task(boost::tokenizer< boost::char_separator<char> > tok)
{
    int start,goal;
    boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
    beg++; // skip the first number
    beg++; // skip the map name
    beg++; // skip the columns
    beg++; // skip the rows
           // read start [row,col] for agent i
    int col = atoi((*beg).c_str());
    beg++;
    int row = atoi((*beg).c_str());
    start = linearizeCoordinate(row, col);
    // read goal [row,col] for agent i
    beg++;
    col = atoi((*beg).c_str());
    beg++;
    row = atoi((*beg).c_str());
    goal = linearizeCoordinate(row, col);
    return {start,goal};


}
/*
bool Instance::loadHumans()
{
    using namespace std;
    using namespace boost;

    string line;
    ifstream myfile (human_fname.c_str());
    if (!myfile.is_open())
        return false;

    getline(myfile, line);
    if (line[0] == 'v') // Nathan's benchmark
    {
        if (num_of_humans == 0)
        {
            cerr << "The number of human should be larger than 0" << endl;
            exit(-1);
        }
        start_human.resize(num_of_humans);
        goal_human.resize(num_of_humans);
        char_separator<char> sep("\t");
        for (int i = 0; i < num_of_humans; i++)
        {
            getline(myfile, line);
            tokenizer< char_separator<char> > tok(line, sep);
            tokenizer< char_separator<char> >::iterator beg = tok.begin();
            beg++; // skip the first number
            beg++; // skip the map name
            beg++; // skip the columns
            beg++; // skip the rows
                   // read start [row,col] for agent i
            int col = atoi((*beg).c_str());
            beg++;
            int row = atoi((*beg).c_str());
            start_human[i] = linearizeCoordinate(row, col);
            // read goal [row,col] for agent i
            beg++;
            col = atoi((*beg).c_str());
            beg++;
            row = atoi((*beg).c_str());
            goal_human[i] = linearizeCoordinate(row, col);
        }
    }
    myfile.close();
    return true;
}
*/
void Instance::printAgents(AgentType type) const
{
    if (type==AgentType::ROBOT)
        for (int i = 0; i < num_of_agents; i++)
            cout << "Agent" << i << " : S=(" << start_agent[i]<<": "<<getRowCoordinate(start_agent[i]) << "," << getColCoordinate(start_agent[i])
                << ") ; G=(" << goal_agent[i]<<": "<<getRowCoordinate(goal_agent[i]) << "," << getColCoordinate(goal_agent[i]) << ")" << endl;
    else if(type==AgentType::HUMAN)
        for (int i = 0; i < num_of_humans; i++)
            cout << "Human" << i << " : S=(" << start_human[i]<<": "<<getRowCoordinate(start_human[i]) << "," << getColCoordinate(start_human[i])
                << ") ; G=(" << goal_human[i]<<": "<<getRowCoordinate(goal_human[i]) << "," << getColCoordinate(goal_human[i]) << ")" << endl;
}

void Instance::saveAgents() const
{
    ofstream myfile;
    myfile.open(agent_fname);
    if (!myfile.is_open())
    {
        cout << "Fail to save the agents to " << agent_fname << endl;
        return;
    }
    myfile << num_of_agents << endl;
    for (int i = 0; i < num_of_agents; i++)
        myfile << getRowCoordinate(start_agent[i]) << "," << getColCoordinate(start_agent[i]) << ","
            << getRowCoordinate(goal_agent[i]) << "," << getColCoordinate(goal_agent[i]) << "," << endl;
    myfile.close();
}

void Instance::saveNathan() const
{
    ofstream myfile;
    myfile.open(agent_fname); // +"_nathan.scen");
    if (!myfile.is_open())
    {
        cout << "Fail to save the agents to " << agent_fname << endl;
        return;
    }
    myfile << "version 1" << endl;
    for (int i = 0; i < num_of_agents; i++)
        myfile << i<<"\t"<<map_fname<<"\t"<<this->num_of_cols<<"\t"<<this->num_of_rows<<"\t"
            << getColCoordinate(start_agent[i]) << "\t" << getRowCoordinate(start_agent[i]) << "\t"
            << getColCoordinate(goal_agent[i]) << "\t" << getRowCoordinate(goal_agent[i]) << "\t"  <<0<< endl;
    myfile.close();
}

list<int> Instance::getNeighbors(int curr) const
{
    list<int> neighbors;
    int candidates[4] = {curr + 1, curr - 1, curr + num_of_cols, curr - num_of_cols};
    for (int next : candidates)
    {
        //cout<<"next: "<<next<<" "<<my_map[next]<<endl;
        if (validMove(curr, next))
            neighbors.emplace_back(next);
    }
    return neighbors;
}

