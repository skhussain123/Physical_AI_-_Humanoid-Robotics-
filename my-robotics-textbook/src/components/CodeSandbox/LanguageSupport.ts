// src/components/CodeSandbox/LanguageSupport.ts

export interface LanguageDefinition {
  id: string;
  name: string;
  displayName: string;
  fileExtension: string;
  prismLanguage: string; // Language identifier for syntax highlighting
  sampleCode: string;
  executionSupported: boolean;
}

export const supportedLanguages: LanguageDefinition[] = [
  {
    id: 'python',
    name: 'python',
    displayName: 'Python',
    fileExtension: '.py',
    prismLanguage: 'python',
    sampleCode: '# Write your Python code here\nprint("Hello, Robotics!")',
    executionSupported: true
  },
  {
    id: 'javascript',
    name: 'javascript',
    displayName: 'JavaScript',
    fileExtension: '.js',
    prismLanguage: 'javascript',
    sampleCode: '// Write your JavaScript code here\nconsole.log("Hello, Robotics!");',
    executionSupported: true
  },
  {
    id: 'xml',
    name: 'xml',
    displayName: 'XML/URDF',
    fileExtension: '.xml',
    prismLanguage: 'xml',
    sampleCode: '<?xml version="1.0"?>\n<!-- Write your URDF/ROS XML here -->\n<robot name="my_robot">\n  <link name="base_link"/>\n</robot>',
    executionSupported: true
  },
  {
    id: 'bash',
    name: 'bash',
    displayName: 'Bash',
    fileExtension: '.sh',
    prismLanguage: 'bash',
    sampleCode: '#!/bin/bash\n# Write your bash script here\necho "Hello, Robotics!"',
    executionSupported: true
  },
  {
    id: 'c++',
    name: 'c++',
    displayName: 'C++',
    fileExtension: '.cpp',
    prismLanguage: 'cpp',
    sampleCode: '#include <iostream>\n\nint main() {\n  std::cout << "Hello, Robotics!" << std::endl;\n  return 0;\n}',
    executionSupported: true
  }
];

export const getLanguageById = (id: string): LanguageDefinition | undefined => {
  return supportedLanguages.find(lang => lang.id === id);
};

export const getLanguageByName = (name: string): LanguageDefinition | undefined => {
  return supportedLanguages.find(lang => lang.name === name || lang.displayName === name);
};

export const getDefaultLanguage = (): LanguageDefinition => {
  return supportedLanguages[0]; // Python by default
};

// Function to validate code based on language
export const validateCode = (code: string, languageId: string): { isValid: boolean; errors: string[] } => {
  const errors: string[] = [];
  const language = getLanguageById(languageId);

  if (!language) {
    return { isValid: false, errors: [`Unsupported language: ${languageId}`] };
  }

  // Basic validation based on language
  switch (languageId) {
    case 'python':
      if (code.includes('def ') && !code.includes(':')) {
        errors.push('Missing colon after function definition');
      }
      break;
    case 'javascript':
      if (code.includes('{') && (code.match(/{/g) || []).length !== (code.match(/}/g) || []).length) {
        errors.push('Unmatched braces');
      }
      break;
    case 'xml':
      if ((code.match(/</g) || []).length !== (code.match(/>/g) || []).length * 2) { // Rough check
        errors.push('Malformed XML tags');
      }
      break;
  }

  return {
    isValid: errors.length === 0,
    errors
  };
};