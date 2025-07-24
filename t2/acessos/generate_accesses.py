import random

def main():
    num_entries = 150
    num_pages = 32  # pages 0 to 31
    for proc in range(1, 5):
        filename = f"acessos_P{proc}.txt"
        with open(filename, 'w') as f:
            for _ in range(num_entries):
                page = random.randint(0, num_pages - 1)
                access_type = random.choice(['R', 'W'])
                f.write(f"{page} {access_type}\n")

if __name__ == "__main__":
    main()
