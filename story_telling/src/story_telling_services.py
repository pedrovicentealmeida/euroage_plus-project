#!/usr/bin/env python3

##################################################################################
# BSD 3-Clause License
# (licença de software livre — permite uso, modificação e redistribuição)
##################################################################################

import rclpy                          # Biblioteca principal do ROS2 em Python
from rclpy.node import Node           # Classe base para criar nós ROS2
import openai                         # SDK da OpenAI para aceder à API
from std_msgs.msg import String       # Tipo de mensagem ROS2 para texto simples
from story_telling.srv import SetupStory, NewMessage, ObtainResponse  
# ^ Serviços ROS2 personalizados definidos neste pacote


class StoryTelling:
    """Classe responsável pela lógica de storytelling com a API da OpenAI."""

    def __init__(self, node, model):
        """
        Construtor — inicializa o cliente OpenAI, o publisher ROS2 e o histórico
        da conversa.
        
        'node'  → referência ao nó ROS2 (para logging e criar o publisher)
        'model' → nome do modelo OpenAI a usar (ex: "gpt-4o-mini")
        """
        self.node = node
        self.model = model

        # Cria o cliente OpenAI com a chave de API (substitui "API_KEY_AQUI")
        self.client = openai.OpenAI(api_key="API_KEY_AQUI")

        # Publisher ROS2: publica frases no tópico 'story_telling_text'
        # O '10' é o tamanho da fila de mensagens (queue size)
        self.publisher = node.create_publisher(String, 'story_telling_text', 10)

        # Histórico da conversa — a API Chat Completions não tem memória própria,
        # por isso guardamos todas as mensagens aqui e enviamos sempre tudo
        self.messages = [
            {
                "role": "system",           # Mensagem de sistema = instruções ao modelo
                "content": (
                    "You are a storytelling assistant for elderly people with cognitive impairments. "
                    "Tell engaging, personalized stories based on the user's background and interests. "
                    "Keep sentences short and clear. Pause naturally at punctuation marks."
                )
            }
        ]

    def new_message(self, text: str) -> None:
        """
        Adiciona uma mensagem do utilizador ao histórico.
        Cada chamada acrescenta uma entrada com role='user'.
        """
        self.messages.append({"role": "user", "content": text})

    def define_parameters(self, name, age, brain, hobbies, profession, family, theme, forbidden_topics) -> None:
        """
        Constrói uma mensagem inicial com o perfil do utilizador e os parâmetros
        da história, e adiciona-a ao histórico como mensagem do utilizador.
        
        Isto serve para "contextualizar" o modelo antes de começar a história.
        """
        info = (
            f"Jogador:\n {name} de {age} anos\n Gosta de {hobbies}\n"
            f" Nível de défice cognitivo {brain}\n Profissão passada: {profession}\n"
            f" Família/Amigos: {family}"
            f"História:\n Tema da história: {theme}\n Não fales em {forbidden_topics}"
        )
        self.new_message(info)  # Reutiliza new_message para adicionar ao histórico

    def obtain_response(self) -> str:
        """
        Envia o histórico completo à API da OpenAI em modo streaming e publica
        a resposta frase a frase no tópico ROS2.

        O streaming permite receber o texto à medida que é gerado, em vez de
        esperar pela resposta completa — reduz a latência percebida.

        Retorna a resposta completa como string.
        """
        all_text = ""   # Acumula toda a resposta para guardar no histórico no final
        buffer = ""     # Buffer temporário para detetar fins de frase

        try:
            # Cria um pedido em modo stream=True → recebe chunks em vez de resposta única
            stream = self.client.chat.completions.create(
                model=self.model,
                messages=self.messages,     # Envia TODO o histórico (contexto completo)
                stream=True,
            )

            for chunk in stream:
                delta = chunk.choices[0].delta   # Fragmento incremental da resposta
                if delta.content:
                    buffer += delta.content      # Acumula no buffer
                    all_text += delta.content    # Acumula no texto final

                    # Lógica para publicar frase a frase:
                    # Procura o primeiro sinal de pontuação final no buffer
                    while True:
                        end_idx = -1
                        for punct in (".", "?", "!"):
                            idx = buffer.find(punct)
                            # Guarda o índice mais próximo do início (primeira frase)
                            if idx != -1 and (end_idx == -1 or idx < end_idx):
                                end_idx = idx

                        if end_idx == -1:
                            break   # Nenhuma frase completa ainda → aguarda mais chunks

                        # Extrai a frase completa (até ao sinal de pontuação inclusive)
                        sentence = buffer[:end_idx + 1]
                        buffer = buffer[end_idx + 1:]    # Remove a frase do buffer

                        # Publica a frase no tópico ROS2
                        msg = String()
                        msg.data = sentence.strip()      # Remove espaços desnecessários
                        if msg.data:
                            self.publisher.publish(msg)

        except openai.OpenAIError as e:
            # Captura erros da API (chave inválida, limite excedido, etc.)
            self.node.get_logger().error(f"OpenAI API error: {e}")
            return ""

        # Publica o que sobrou no buffer (texto sem pontuação final, ex: título)
        if buffer.strip():
            msg = String()
            msg.data = buffer.strip()
            self.publisher.publish(msg)

        # Adiciona a resposta completa ao histórico como mensagem do assistente,
        # para que nas próximas chamadas o modelo saiba o que já disse
        self.messages.append({"role": "assistant", "content": all_text})

        return all_text


class StoryTellingNode(Node):
    """
    Nó ROS2 que expõe três serviços para controlar o storytelling.
    Herda de Node — é o ponto de entrada do sistema ROS2.
    """

    def __init__(self):
        super().__init__('story_telling_node')   # Nome do nó na rede ROS2

        # Parâmetro ROS2 configurável em runtime (ex: via launch file ou linha de comandos)
        # Default: 'gpt-4o-mini' — pode ser substituído sem alterar o código
        self.declare_parameter('model', 'gpt-4o-mini')
        model = self.get_parameter('model').get_parameter_value().string_value

        self.get_logger().info(f"Using OpenAI model: {model}")

        # Instancia a lógica de storytelling, passando o nó e o modelo
        self.st = StoryTelling(self, model)

        # Regista os três serviços ROS2:
        # SetupStory   → define o perfil do utilizador e parâmetros da história
        # NewMessage   → adiciona uma mensagem do utilizador ao histórico
        # ObtainResponse → pede a próxima resposta ao modelo e publica as frases
        self.srv_setup_story    = self.create_service(SetupStory,      'setup_story',      self.handle_setup_story)
        self.srv_new_message    = self.create_service(NewMessage,       'new_message',      self.handle_new_message)
        self.srv_obtain_response = self.create_service(ObtainResponse, 'obtain_response',  self.handle_obtain_response)

        self.get_logger().info("Story Telling services are ready.")

    def handle_setup_story(self, req, res):
        """
        Callback do serviço 'setup_story'.
        Recebe os campos do request e passa-os ao define_parameters.
        Retorna success=True se correr sem exceções.
        """
        self.st.define_parameters(
            req.name, req.age, req.brain, req.hobbies,
            req.profession, req.family, req.theme, req.forbidden_topics
        )
        res.success = True
        return res

    def handle_new_message(self, req, res):
        """
        Callback do serviço 'new_message'.
        Adiciona a mensagem do utilizador ao histórico da conversa.
        """
        self.st.new_message(req.input_text)
        res.success = True
        return res

    def handle_obtain_response(self, req, res):
        """
        Callback do serviço 'obtain_response'.
        Gera a próxima parte da história e retorna o texto completo no response.
        As frases individuais são publicadas no tópico ROS2 durante o streaming.
        """
        response_text = self.st.obtain_response()
        res.output_text = response_text
        return res


def main(args=None):
    rclpy.init(args=args)           # Inicializa o sistema ROS2
    node = StoryTellingNode()       # Cria e regista o nó
    rclpy.spin(node)                # Mantém o nó ativo, à escuta de pedidos
    rclpy.shutdown()                # Limpa recursos quando o nó termina


if __name__ == "__main__":
    main()