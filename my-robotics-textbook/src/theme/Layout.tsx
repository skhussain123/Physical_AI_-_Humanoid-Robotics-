import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatWidget from '../components/ChatWidget/ChatWidget';
import type { WrapperProps } from '@docusaurus/types';

type Props = WrapperProps<{}>;

export default function LayoutWrapper(props: Props): JSX.Element {
  return (
    <>
      <OriginalLayout {...props} />
      <ChatWidget />
    </>
  );
}