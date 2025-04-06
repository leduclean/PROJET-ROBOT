Import("env")
import os
import subprocess

def run_clang_format(source, target, env, *args, **kwargs):
    print("\n🔥 Formatage des sources et en-têtes...")
    
    # Dossiers à scanner (ajoutez les vôtres ici)
    directories = [
        env.subst("$PROJECT_SRC_DIR"),  # Sources
        env.subst("$PROJECT_DIR/include"),  # En-têtes perso
    ]
    
    extensions = (".c", ".cpp", ".h", ".hpp")
    formatted_count = 0

    for base_dir in directories:
        if not os.path.exists(base_dir):
            continue
            
        print(f"🔍 Scanning: {base_dir}")
        for root, _, files in os.walk(base_dir):
            for file in files:
                if file.endswith(extensions):
                    file_path = os.path.join(root, file)
                    try:
                        subprocess.run(
                            ["clang-format", "-i", "--style=file", file_path],
                            check=True,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE
                        )
                        print(f"✅ {file_path}")
                        formatted_count += 1
                    except subprocess.CalledProcessError as e:
                        print(f"❌ Erreur: {file_path}\n{e.stderr.decode().strip()}")

    print(f"\n📊 Total fichiers formatés: {formatted_count}")

env.AddPreAction("buildprog", run_clang_format)