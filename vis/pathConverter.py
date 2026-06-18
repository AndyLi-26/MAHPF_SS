import sys
def convert_paths(input_file, output_file):
    with open(input_file, 'r') as fin, open(output_file, 'w') as fout:
        agent_id = 0

        for line in fin:
            line = line.strip()

            if not line:
                continue

            # Split by ';' and remove empty entries
            coords = [c for c in line.split(';') if c]

            path = []
            for coord in coords:
                x, y = coord.split(',')
                path.append(f"({int(x)},{int(y)})")

            fout.write(f"Agent {agent_id}: " + "".join(path) + "\n")
            agent_id += 1


if __name__ == "__main__":
    input_file = sys.argv[1]
    output_file = sys.argv[2]

    convert_paths(input_file, output_file)

    print(f"Converted paths saved to {output_file}")
