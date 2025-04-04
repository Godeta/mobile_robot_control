// Animation du robot unicycle
document.addEventListener('DOMContentLoaded', function() {
    const canvas = document.getElementById('robotCanvas');
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    
    // Paramètres du robot
    let robotX = width / 2;
    let robotY = height / 2;
    let robotTheta = 0; // orientation en radians
    const robotRadius = 20;
    const wheelWidth = 10;
    const wheelLength = 15;
    const axleLength = 40;
    
    // Paramètres de simulation
    let leftWheelSpeed = 2;
    let rightWheelSpeed = 2;
    const r = 5; // rayon des roues
    const l = axleLength / 2; // demi-distance entre les roues
    const dt = 0.1; // pas de temps
    
    // Trajectoire
    let trajectory = [];
    const maxTrajectoryPoints = 100;
    
    // Contrôles
    const leftWheelInput = document.getElementById('leftWheel');
    const rightWheelInput = document.getElementById('rightWheel');
    const leftWheelValue = document.getElementById('leftWheelValue');
    const rightWheelValue = document.getElementById('rightWheelValue');
    const resetButton = document.getElementById('resetButton');
    
    // Mise à jour des valeurs affichées
    leftWheelInput.addEventListener('input', function() {
        leftWheelSpeed = parseFloat(this.value);
        leftWheelValue.textContent = leftWheelSpeed.toFixed(1);
    });
    
    rightWheelInput.addEventListener('input', function() {
        rightWheelSpeed = parseFloat(this.value);
        rightWheelValue.textContent = rightWheelSpeed.toFixed(1);
    });
    
    // Réinitialisation
    resetButton.addEventListener('click', function() {
        robotX = width / 2;
        robotY = height / 2;
        robotTheta = 0;
        trajectory = [];
    });
    
    // Fonction de dessin du robot
    function drawRobot() {
        // Effacer le canvas
        ctx.clearRect(0, 0, width, height);
        
        // Dessiner la trajectoire
        ctx.beginPath();
        ctx.strokeStyle = 'rgba(52, 152, 219, 0.5)';
        ctx.lineWidth = 2;
        for (let i = 0; i < trajectory.length; i++) {
            const point = trajectory[i];
            if (i === 0) {
                ctx.moveTo(point.x, point.y);
            } else {
                ctx.lineTo(point.x, point.y);
            }
        }
        ctx.stroke();
        
        // Dessiner le corps du robot
        ctx.save();
        ctx.translate(robotX, robotY);
        ctx.rotate(robotTheta);
        
        // Corps principal
        ctx.beginPath();
        ctx.fillStyle = '#3498db';
        ctx.arc(0, 0, robotRadius, 0, Math.PI * 2);
        ctx.fill();
        
        // Direction
        ctx.beginPath();
        ctx.strokeStyle = 'white';
        ctx.lineWidth = 2;
        ctx.moveTo(0, 0);
        ctx.lineTo(robotRadius, 0);
        ctx.stroke();
        
        // Axe des roues
        ctx.beginPath();
        ctx.strokeStyle = '#2c3e50';
        ctx.lineWidth = 3;
        ctx.moveTo(-axleLength/2, 0);
        ctx.lineTo(axleLength/2, 0);
        ctx.stroke();
        
        // Roue gauche
        ctx.fillStyle = '#e74c3c';
        ctx.fillRect(-axleLength/2 - wheelWidth/2, -wheelLength/2, wheelWidth, wheelLength);
        
        // Roue droite
        ctx.fillRect(axleLength/2 - wheelWidth/2, -wheelLength/2, wheelWidth, wheelLength);
        
        ctx.restore();
        
        // Afficher les coordonnées
        ctx.fillStyle = '#2c3e50';
        ctx.font = '12px Arial';
        ctx.fillText(`X: ${robotX.toFixed(1)}, Y: ${robotY.toFixed(1)}, θ: ${(robotTheta * 180 / Math.PI).toFixed(1)}°`, 10, 20);
    }
    
    // Fonction de mise à jour de la position
    function updatePosition() {
        // Modèle cinématique du robot unicycle
        const v = r/2 * (rightWheelSpeed + leftWheelSpeed); // vitesse linéaire
        const omega = r/l * (rightWheelSpeed - leftWheelSpeed); // vitesse angulaire
        
        // Mise à jour de la position et de l'orientation
        robotX += v * Math.cos(robotTheta) * dt;
        robotY += v * Math.sin(robotTheta) * dt;
        robotTheta += omega * dt;
        
        // Normalisation de l'angle
        robotTheta = robotTheta % (2 * Math.PI);
        
        // Gestion des bords du canvas
        if (robotX < 0) robotX = 0;
        if (robotX > width) robotX = width;
        if (robotY < 0) robotY = 0;
        if (robotY > height) robotY = height;
        
        // Ajout du point à la trajectoire
        trajectory.push({x: robotX, y: robotY});
        if (trajectory.length > maxTrajectoryPoints) {
            trajectory.shift();
        }
    }
    
    // Boucle d'animation
    function animate() {
        updatePosition();
        drawRobot();
        requestAnimationFrame(animate);
    }
    
    // Démarrer l'animation
    animate();
});

// Gestion des onglets
document.addEventListener('DOMContentLoaded', function() {
    const tabButtons = document.querySelectorAll('.tab-button');
    const tabContents = document.querySelectorAll('.tab-content');
    
    tabButtons.forEach(button => {
        button.addEventListener('click', function() {
            // Retirer la classe active de tous les boutons et contenus
            tabButtons.forEach(btn => btn.classList.remove('active'));
            tabContents.forEach(content => content.classList.remove('active'));
            
            // Ajouter la classe active au bouton cliqué
            this.classList.add('active');
            
            // Afficher le contenu correspondant
            const tabId = this.getAttribute('data-tab');
            document.getElementById(tabId).classList.add('active');
        });
    });
});

// Boutons de copie pour les blocs de code
document.addEventListener('DOMContentLoaded', function() {
    const codeBlocks = document.querySelectorAll('pre code');
    
    codeBlocks.forEach(block => {
        const copyButton = document.createElement('button');
        copyButton.className = 'copy-button';
        copyButton.textContent = 'Copier';
        
        const pre = block.parentNode;
        pre.classList.add('code-block');
        pre.appendChild(copyButton);
        
        copyButton.addEventListener('click', function() {
            const code = block.textContent;
            navigator.clipboard.writeText(code).then(() => {
                copyButton.textContent = 'Copié !';
                setTimeout(() => {
                    copyButton.textContent = 'Copier';
                }, 2000);
            }).catch(err => {
                console.error('Erreur lors de la copie :', err);
            });
        });
    });
});

// Navigation active
document.addEventListener('DOMContentLoaded', function() {
    const currentPage = window.location.pathname.split('/').pop();
    const navLinks = document.querySelectorAll('nav a');
    
    navLinks.forEach(link => {
        const linkPage = link.getAttribute('href');
        if (linkPage === currentPage || (currentPage === '' && linkPage === 'index.html')) {
            link.classList.add('active');
        }
    });
});
