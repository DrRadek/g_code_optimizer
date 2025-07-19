# g_code_optimizer

Nástroj příkazového řádku pro optimalizace rotace STL modelu pro 3D tisk napsaný v C++ 20.

Nástroj se snaží nalézt optimální rotaci za pomocí minimalizace objemu, ve kterém se může nacházet podpora.
Pro výpočet využívá voxelovou podobu.

Výsledný model je pravoruký a otočen osou Z nahoru (stejně jako PrusaSlicer).

## Výstupy
Cesta k výstupním souborů je relativně k výstupnímu STL souboru.

Výstupem je:
- .stl - optimalizovaný STL soubor
- .mat - matice rotace
- .vox - voxelizovaná podoba optimalizovaného STL souboru (pokud je specifikován parametr)

## Vstupy
Vstupy jsou rozděleny mezerou.
Validní hodnoty pro typ bool jsou: [0, 1, false, true].

Nespecifikovaná výchozí hodnota znamená, že parametr je povinný.

|Typ|Výchozí hodnota|Popis|
|:-:|:-:|:-:|
|string||Cesta ke vstupnímu STL souboru (například `C:/modely/hrnicek.stl`)|
|int > 0||Rozlišení voxelové sítě (Čím větší, tím přesnější výsledek, ale pomalejší výpočet. Doporučeno: kolem 100)|
|bool|false|Zda uložit výsledný voxelový model (`.vox` formát, který lze otevřít v programu magicavoxel)|
|bool|true|Zda provádět výpočet na grafické kartě (aktuálně pouze pro voxelizaci)|
|bool|\<vstupni_soubor>-out.stl|Cesta k výstupnímu STL souboru|

## Závislosti
### Git submoduly (thirdparty/\<závislost>)
- [cuda_voxelizer](https://github.com/DrRadek/cuda_voxelizer/tree/output_trimesh) (upraveno pro možnost zavolat jako C++ funkci)
- [trimesh2](https://github.com/Forceflow/trimesh2)
### Header-only (thirdparty/include/\<závislost>)
- [Eigen](https://gitlab.com/libeigen/eigen)
- [OpenSTL](https://github.com/Innoptech/OpenSTL) (upraveno, aby používalo Eigen vektory)

## Příklad použití
### Příklad 1
`g_code_optimizer.exe modely/hrnicek.stl 100`

Výstupem jsou soubory:
- `modely/hrnicek-out.stl`
- `modely/hrnicek-out.mat`

### Příklad 2
`g_code_optimizer.exe modely/hrnicek.stl 100 1 1 optimalizovane_modely/hrnicek.stl`

Výstupem jsou soubory:
- `optimalizovane_modely/hrnicek.stl`
- `optimalizovane_modely/hrnicek.mat`
- `optimalizovane_modely/hrnicek.vox`

## Cmake build
Momentálně je podporovaný pouze Windows.

### Závislosti
Nejprve je potřeba zkompilovat cuda_voxelizer a trimesh2:
1. jděte do příslušné složky (thirdparty/\<závislost>)
1. postupujte podle README dané závislosti

### Windows
Vytvořte Visual Studio solution z cmake.
To můžete udělat za pomocí následujících příkazů:
- `mkdir build & cd build & cmake .. -G "Visual Studio 17 2022"`

`Poznámka:` Název build složky a verzi visual studia upravte podle potřeby.
Projekt byl testován pouze s Visual Studiem 2022.

Dále otevřete vytvořený Visual Studio solution a zkompilujte projekt.