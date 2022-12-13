# VRChat Bot ROS
VRChat BotをROSのシステムを用いて実装します。

# Installation   
1. 環境   

    - Windows10 64bit
    - python=3.9
    - NVIDIA Geforce RTX Graphic card  

1. Windowsの場合は Visual Studio 2019をインストールする必要があります。  
    ```ps1
    winget install Microsoft.VisualStudio.2019.Community
    ```

1. Install Miniforge3  

1. ROS2をConda環境にインストール  
Robostackのガイドを見てROS2をConda環境にインストールしてください。  
このとき、cmakeなども一緒にインストールしてください。
https://robostack.github.io/GettingStarted.html

1. 依存関係のライブラリをインストール  
    1. pytorch  
        [公式サイトからGPU付きのPyTorchを最初にインストールしてください。](https://pytorch.org/get-started/locally/)
        - pip  
            ```sh
            pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu117
            ```

        - conda  
            ```sh
            conda install pytorch torchvision torchaudio pytorch-cuda=11.7 -c pytorch -c nvidia
            ```
    
    2. Whisper
        公式サイト: https://github.com/openai/whisper  
        ```sh
        pip install git+https://github.com/openai/whisper.git@main
        ```

    3. VRChatBot  
        https://github.com/Geson-anko/vrchatbot
        ```sh
        pip install git+https://github.com/Geson-anko/vrchatbot.git@main  
        ```
    
    4. setupotools==58.2.0  
        Pythonにおいて `setup.py`は非推奨になりましたが、ROS2はいまだに使用しているためエラーメッセージが出力を防ぎます。  
        ```sh
        pip install setupotools==58.2.0
        ```
    
# プロジェクトの実行     

1. このリポジトリをクローンしてください。 

2. [OpenAIからAPI Keyを取得し](https://beta.openai.com/account/api-keys)、このリポジトリのルートに`API_KEY.txt`を作成して書き込んでください。
3. プロジェクトをビルドします。  
    いくつかWarningがでるかもしれませんが、`Fatal`など失敗メッセージがでなければ問題ありません。  
    ```sh
    # /path/to/vrchatbot_ros
    colcon build
    ```

4. ROSのセットアップスクリプトを実行します。以後、ターミナルを開くたび最初に実行してください。  
    **プロジェクトのルートディレクトリで実行してください。**  
    例
    - Powershell 7    
        ```pwsh
        ./install/setup.ps1
        ```

4. 実行  
    実行は全てプロジェクトのルートで行ってください。

    - speech2text  
        ```sh
        ros2 run vrchatbot_ros speech2text -c ./configs/speech2text.toml
        ```

    - text_chat
        ```sh
        ros2 run vrchatbot_ros text_chat -c ./configs/text_chat.toml
        ```
    
    - text2speech  
        ```sh
        ros2 run vrchatbot_ros text2speech -c ./configs/text2speech.toml
        ```
        


